#!/usr/bin/env python3
"""Fixed-window sampler: entry, deadline, wrap, degeneracy and transition tests."""
import math
import unittest

from front_marker_control.refinement import (
    DEFAULTS, FinalPoseRefiner, PoseTransition, circular_mean, circular_std,
    validate_parameters,
)
from front_marker_control.localization import LocalPoseBuffer, yaw_pose
from front_marker_control.control import wrap_angle


def radians(values):
    return [math.radians(v) for v in values]


class RefinementTest(unittest.TestCase):
    def setUp(self):
        self.parameters = dict(DEFAULTS)
        self.engine = FinalPoseRefiner(self.parameters, (0.8, 0, 0), (0.12, 0.08, 0.08), 0.1, 0.1)
        self.buffer = LocalPoseBuffer(500)
        self.now = 10.0

    def pose(self, stamp, position=(0, 0, 0), yaw=0):
        pose = yaw_pose(stamp, position, yaw)
        self.assertIsNone(self.buffer.accept(stamp, 'odom', pose.position, pose.quaternion))
        return pose

    def start(self):
        # XYZ 容差即入场：无速度门、无保持时长、无 yaw 要求。
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 5.0, (0, 0, 0), self.buffer)
        self.now += 1/30
        self.assertEqual(self.engine.mode, 'YAW_SAMPLING')

    def sample(self, yaw=0, body=(0.8, 0, 0), position=(0, 0, 0), body_yaw=0):
        pose = self.pose(self.now, position, body_yaw)
        self.engine.marker(self.now, self.now, body, yaw)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
        self.now += 1/30

    def target(self, yaw=0):
        self.start()
        duration = self.parameters['yaw_sampling_duration_sec']
        steps = int(duration * 30) - 1  # 停在截止 tick（第 90 步）之前
        for _ in range(steps):
            self.sample(yaw)
        # 第 90 步（elapsed == 3.0 s）触发截止提交。
        self.sample(yaw)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')

    def test_entry_on_tolerance_even_at_speed(self):
        # XYZ 在容差内但速度超限：立即开始测量。
        self.start()
        self.assertEqual(self.engine.window_start, 10.0)

    def test_entry_rejects_outside_tolerance(self):
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0.13, 0, 0), self.buffer)
        self.assertEqual(self.engine.mode, 'ALIGN')

    def test_entry_with_vision_yaw_error(self):
        # 入场不要求 yaw 已到位：误差 40° 也可以进入采样。
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 0.2, (0, 0, 0), self.buffer)
        self.assertEqual(self.engine.mode, 'YAW_SAMPLING')

    def test_entry_rejects_stale_vision_error(self):
        # 视觉超时但旧误差在容差内：不得凭旧误差启动采样。
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, False, pose, 0, (0, 0, 0), self.buffer)
        self.assertEqual(self.engine.mode, 'ALIGN')

    def test_deadline_excludes_late_samples(self):
        # 源/接收时间晚于截止的样本不得进入最终均值（即使先于控制
        # 定时器到达）。
        self.start()
        for i in range(80):
            self.sample()
        late = self.engine.window_start + 3.01
        self.engine.marker(late, late, (0.8, 0, 0), math.radians(40))
        deadline = self.engine.window_start + 3.0
        # 80 个样本后 self.now 已越过 deadline；直接用当前时刻提交。
        self.now = max(self.now, deadline)
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        # 迟到样本被拒：yaw 均值仍为 0 而不是被 40° 拉偏。
        self.assertAlmostEqual(self.engine.target[1], 0.0, places=6)
        self.assertFalse(self.engine.pending)

    def test_unpairable_head_does_not_block_followers(self):
        # 队首落在 VIO 历史间隙：丢弃该帧，后续可配对样本照常进入
        # 目标计算，窗口截止不再漏收或误报 empty_window。
        self.start()
        # 构造历史间隙：odom 样本间有 > max_gap 的空洞。
        gap_start = self.now
        for _ in range(30):
            self.sample()
        # 在 newest 之后留一个空洞（跳 0.5 s 再继续 odom），空洞前的
        # marker 时间戳落在间隙内（无法插值），其后再来两个正常样本。
        stale_marker_stamp = self.now
        self.now += 0.5
        # 让 newest_stamp 越过 stale_marker_stamp：该样本永远无法配对。
        self.pose(self.now)
        self.now += 1/30
        self.engine.marker(stale_marker_stamp, stale_marker_stamp, (0.8, 0, 0), math.radians(40))
        n_before = len(self.engine.samples)
        for _ in range(3):
            self.sample()
        self.engine.process_pairs(self.now, self.buffer)
        self.assertEqual(self.engine.mode, 'YAW_SAMPLING')
        # 后续样本被处理，且未产生乱序样本。
        self.assertGreater(len(self.engine.samples), n_before)
        self.assertFalse(self.engine.pending)
        # 窗口照常截止并提交目标（非 empty_window）。
        deadline = self.engine.window_start + 3.0
        self.now = max(self.now, deadline)
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertIsNotNone(self.engine.target)

    def test_window_survives_frame_loss_speed_spike_and_drift(self):
        # 断帧 + 速度尖峰 + XYZ 越界均不清零、不重启窗口。
        self.start()
        start = self.engine.window_start
        for i in range(30):
            if i == 10:
                self.now += 0.3  # 300 ms 断流（旧逻辑 10 倍于清零阈值）
            if i == 20:
                pose = self.pose(self.now)
                self.engine.tick(self.now, 1/30, True, pose, 0.5, (0.5, 0, 0), self.buffer)
                self.now += 1/30
                continue
            self.sample()
        self.assertEqual(self.engine.mode, 'YAW_SAMPLING')
        self.assertEqual(self.engine.window_start, start)
        self.assertGreater(len(self.engine.samples), 25)

    def test_vision_loss_does_not_pause_or_reset_window(self):
        self.start()
        for i in range(40):
            pose = self.pose(self.now)
            fresh = (i % 2 == 0)
            self.engine.tick(self.now, 1/30, fresh, pose, 0, (0, 0, 0), self.buffer)
            self.now += 1/30
        self.assertEqual(self.engine.mode, 'YAW_SAMPLING')
        self.assertEqual(self.engine.window_start, 10.0)

    def test_deadline_fires_without_new_images(self):
        # 截止判断在控制定时器：没有新图像窗口照常结束。
        self.start()
        # 先完成一次源时间配对（odom 夹住 marker 时间戳）。
        before = self.pose(self.now - 0.001)
        after = self.pose(self.now + 0.001)
        self.engine.marker(self.now, self.now, (0.8, 0, 0), 0)
        self.engine.process_pairs(self.now, self.buffer)
        self.assertEqual(len(self.engine.samples), 1)
        # 之后无任何新图像/新 marker，窗口照常在截止周期提交目标。
        self.now += 3.0
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, False, pose, 0, None, self.buffer)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')

    def test_queued_marker_pairs_even_when_vision_lost(self):
        # 有效 Marker 入队时 VIO 尚未覆盖其时间戳；随后视觉失效但
        # VIO 正常补齐——已入队样本必须完成配对，窗口截止照常提交。
        self.start()
        # 10.03 收到 marker：此时 buffer newest 未越过该时间戳，等待 VIO。
        self.engine.marker(self.now, self.now, (0.8, 0, 0), 0)
        self.assertEqual(len(self.engine.pending), 1)
        # 视觉失效，但 VIO 继续更新并覆盖 marker 时间戳。
        self.now += 1/30
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, False, pose, 0, None, self.buffer)
        self.assertEqual(len(self.engine.samples), 1)  # 已完成配对
        self.assertFalse(self.engine.pending)
        # 窗口截止（无更多视觉）仍提交目标，而非 empty_window。
        deadline = self.engine.window_start + 3.0
        self.now = max(self.now, deadline)
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, False, pose, 0, None, self.buffer)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertIsNotNone(self.engine.target)

    def test_joint_target_rotates_hold_offset(self):
        self.target(math.radians(8))
        p, yaw = self.engine.target
        self.assertAlmostEqual(yaw, math.radians(8))
        self.assertAlmostEqual(p[0], .8-.8*math.cos(yaw))
        self.assertAlmostEqual(p[1], -.8*math.sin(yaw))
        self.assertEqual(self.engine.transition.start, (0, 0, 0))
        self.assertEqual(self.engine.transition.yaw_start, 0)

    def test_moving_body_produces_constant_world_target(self):
        self.start()
        for i in range(95):
            yaw = math.radians(i/20)
            position = (i/1000, 0, 0)
            body = ((.8-position[0])*math.cos(yaw), -(.8-position[0])*math.sin(yaw), 0)
            self.sample(math.radians(8)-yaw, body, position, yaw)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertAlmostEqual(self.engine.target[1], math.radians(8))
        self.assertAlmostEqual(self.engine.target[0][1], -.8*math.sin(math.radians(8)))

    def test_outliers_kept_and_single_sample_usable(self):
        # 不剔除离群点、无样本数门槛：单个样本即可提交目标。
        self.start()
        before = self.pose(self.now - 0.001)
        after = self.pose(self.now + 0.001)
        self.engine.marker(self.now, self.now, (0.8, 0, 0), math.radians(30))
        self.engine.process_pairs(self.now, self.buffer)
        self.assertEqual(len(self.engine.samples), 1)
        self.now += 3.0
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertAlmostEqual(self.engine.target[1], math.radians(30))

    def test_wraparound_mean(self):
        # ±180° 均值必须落在反方向而不是 0。
        self.start()
        for i in range(90):
            self.sample(math.radians(179) if i % 2 else math.radians(-179))
        self.now += 1.0
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertGreater(abs(math.degrees(self.engine.target[1])), 178)

    def test_circular_mean_degenerate_fallback(self):
        # 圆周向量均值长度 < 1e-6：退化为相对入场 yaw 展开的算术平均，
        # 结果确定（这里展开后两角均为 ±180，平均 = reference+180，
        # wrap 后幅值 |mean-reference| = pi）。
        reference = math.radians(10)
        angles = [reference + math.radians(180), reference - math.radians(180)]
        mean = circular_mean(angles, reference)
        self.assertAlmostEqual(abs(wrap_angle(mean - reference)), math.pi, places=6)
        # 相位对消的对称分布：退化路径仍返回确定值。
        self.assertTrue(math.isfinite(circular_mean([0.0, math.pi], 0.0)))

    def test_circular_std_observation_only(self):
        self.assertAlmostEqual(circular_std([0.0, 0.0]), 0.0, places=6)
        self.assertTrue(math.isfinite(circular_std(radians([0, 40, 80]))))

    def test_empty_window_returns_to_align(self):
        self.start()
        # 窗口内零样本：记录 empty_window、回 ALIGN。
        self.now += 3.0
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
        self.assertEqual(self.engine.mode, 'ALIGN')
        self.assertEqual(self.engine.reason, 'empty_window')
        self.assertIsNone(self.engine.target)

    def test_duplicate_and_backwards_stamps_ignored(self):
        self.start()
        self.sample()
        stamp = self.now - 1/30
        for _ in range(30):
            self.engine.marker(stamp, self.now, (0.8, 0, 0), 0)
        self.assertEqual(len(self.engine.samples), 1)

    def test_pair_timeout_drops_frame_not_window(self):
        # 配对失败只丢该帧，不清零窗口。
        self.start()
        self.engine.marker(self.now, self.now, (0.8, 0, 0), 0)
        self.engine.process_pairs(self.now + 0.51, self.buffer)
        self.assertEqual(self.engine.mode, 'YAW_SAMPLING')
        self.assertFalse(self.engine.samples)
        self.assertFalse(self.engine.pending)
        self.assertEqual(self.engine.window_start, 10.0)
        # 之后新时间戳的观测照常收集（同一时间戳已被去重）。
        self.now += 1/30
        self.sample()
        self.assertEqual(len(self.engine.samples), 1)

    def test_unpairable_marker_excluded_from_target(self):
        # odom 未覆盖的 marker 源时间戳不参与计算。
        self.start()
        self.engine.marker(self.now + 5.0, self.now, (0.8, 0, 0), math.radians(20))
        for _ in range(90):
            self.sample()
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertAlmostEqual(self.engine.target[1], 0.0, places=6)

    def test_pre_window_pairs_excluded(self):
        # 早于窗口入场时刻的配对样本不进入目标计算。
        old = self.now - 0.5
        self.pose(old)
        pose = self.pose(self.now)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
        self.engine.marker(old, old, (0.8, 0, 0), math.radians(20))
        self.engine.process_pairs(self.now, self.buffer)
        self.now += 1/30
        for _ in range(90):
            self.sample()
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertAlmostEqual(self.engine.target[1], 0.0, places=6)

    def test_transition_completes_to_pos_hold(self):
        self.target()
        for _ in range(600):
            pose = self.pose(self.now)
            self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
            self.now += 1/30
            if self.engine.mode == 'POS_HOLD':
                break
        self.assertEqual(self.engine.mode, 'POS_HOLD')
        self.assertEqual(self.engine.reason, 'transition complete')
        # 定点期间视觉误差偏移不再触发重新测量。
        for _ in range(300):
            self.sample(math.radians(20), (0.5, 0.2, 0.2))
        self.assertEqual(self.engine.mode, 'POS_HOLD')

    def test_hard_resets_clear_window_and_target(self):
        for reason_call in (
            lambda: self.engine.reset('VIO 失效'),
            lambda: self.engine.reset('FCU gate interrupted'),
        ):
            self.setUp()
            self.target()
            reason_call()
            self.assertEqual(self.engine.mode, 'ALIGN')
            self.assertIsNone(self.engine.target)
            self.assertFalse(self.engine.samples)
            # 恢复后重新满足入场条件才能开新窗口。
            pose = self.pose(self.now)
            self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
            self.assertEqual(self.engine.mode, 'YAW_SAMPLING')
            self.assertEqual(len(self.engine.samples), 0)

    def test_invalid_parameters(self):
        for key, value in (('yaw_sampling_duration_sec', 0),
                           ('pose_transition_speed_mps', float('nan'))):
            p = dict(self.parameters, **{key: value})
            with self.assertRaises(ValueError):
                validate_parameters(p)

    def test_transition_rate_acceleration_and_wrap(self):
        transition = PoseTransition((0, 0, 0), math.radians(179),
                                    ((.3, .4, 0), math.radians(-179)), self.parameters)
        previous = transition.command
        velocity = (0, 0, 0)
        for _ in range(300):
            current = transition.step(.02)
            new_velocity = tuple((b-a)/.02 for a, b in zip(previous[0], current[0]))
            self.assertLessEqual(math.sqrt(sum(v*v for v in new_velocity)), .1+1e-9)
            acceleration = math.sqrt(sum(((b-a)/.02)**2 for a, b in zip(velocity, new_velocity)))
            self.assertLessEqual(acceleration, .5+1e-8)
            self.assertLessEqual(abs(wrap_angle(current[1]-previous[1])), math.radians(10)*.02+1e-9)
            previous, velocity = current, new_velocity
        self.assertTrue(transition.done)
        self.assertEqual(transition.command[0], (.3, .4, 0))
        self.assertAlmostEqual(transition.command[1], math.radians(-179))


if __name__ == '__main__':
    unittest.main()
