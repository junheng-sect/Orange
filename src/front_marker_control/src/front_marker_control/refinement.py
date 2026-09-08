"""Fixed-duration final-pose sampling. Pure Python; caller serializes access.

The refiner runs ALIGN -> YAW_SAMPLING -> POSE_ALIGNING -> POS_HOLD:

- ALIGN ends as soon as the vision XYZ error is inside tolerance. No speed
  gate, no hold time and no yaw requirement.
- YAW_SAMPLING is a fixed wall-clock window of ``yaw_sampling_duration_sec``
  counted by the control timer. Missing frames, speed spikes and XYZ
  deviations never pause, clear or restart it.
- When the timer deadline is reached the first control tick after it
  computes the fixed XYZ+yaw target from every pairable sample collected in
  the window and immediately emits position setpoints (POSE_ALIGNING ->
  POS_HOLD via the smooth transition). No visual acceptance, no
  resampling while holding.
"""
import math
from collections import deque

from front_marker_control.control import wrap_angle
from front_marker_control.geometry import (
    rotate_vector_by_quaternion, rotate_vector_by_yaw, yaw_from_quaternion,
)

DEFAULTS = {
    'yaw_sampling_duration_sec': 3.0,
    'yaw_sample_pending_timeout_sec': 0.50,
    'pose_transition_speed_mps': 0.10,
    'pose_transition_accel_mps2': 0.50,
    'pose_transition_yaw_rate_deg_s': 10.0,
}


def validate_parameters(parameters):
    for name in DEFAULTS:
        if name not in parameters:
            continue
        value = parameters[name]
        if not math.isfinite(value) or value <= 0:
            raise ValueError('%s must be finite and positive' % name)


def circular_mean(angles, reference):
    """Circular mean of ``angles`` unwrapped relative to ``reference``.

    The circular vector mean is ill-conditioned near the +/-180 wrap point
    and undefined when the mean vector length collapses to zero. In both
    cases fall back to the arithmetic mean of angles unwrapped relative to
    ``reference`` so the result stays deterministic.
    """
    cosine = sum(math.cos(a) for a in angles) / len(angles)
    sine = sum(math.sin(a) for a in angles) / len(angles)
    if math.hypot(cosine, sine) < 1e-6:
        unwrapped = [reference + wrap_angle(a - reference) for a in angles]
        return wrap_angle(sum(unwrapped) / len(unwrapped))
    return math.atan2(sine, cosine)


def circular_std(angles):
    """Circular standard deviation in radians; observation only, no gate."""
    if not angles:
        return float('inf')
    cosine = sum(math.cos(a) for a in angles) / len(angles)
    sine = sum(math.sin(a) for a in angles) / len(angles)
    length = min(1.0, math.hypot(cosine, sine))
    return math.sqrt(-2 * math.log(max(length, 1e-15)))


class PoseTransition:
    """Straight-line rest-to-rest trapezoid and independent shortest-path yaw slew."""
    def __init__(self, position, yaw, target, parameters):
        self.start = tuple(position)
        self.yaw_start = yaw
        self.target = target
        self.delta = tuple(b - a for a, b in zip(position, target[0]))
        self.distance = math.sqrt(sum(v*v for v in self.delta))
        self.accel = parameters['pose_transition_accel_mps2']
        self.speed = min(parameters['pose_transition_speed_mps'],
                         math.sqrt(self.distance * self.accel))
        self.ramp = self.speed / self.accel
        self.cruise = max(0.0, self.distance / self.speed - self.ramp) if self.speed else 0.0
        self.duration = 2*self.ramp + self.cruise
        self.yaw_delta = wrap_angle(target[1] - yaw)
        self.yaw_rate = math.radians(parameters['pose_transition_yaw_rate_deg_s'])
        self.time = 0.0
        self.done = False
        self.command = (self.start, yaw)

    def step(self, dt):
        self.time += dt
        t = min(self.time, self.duration)
        if t < self.ramp:
            distance = 0.5*self.accel*t*t
        elif t < self.ramp + self.cruise:
            distance = 0.5*self.speed*self.ramp + self.speed*(t-self.ramp)
        else:
            distance = self.distance - 0.5*self.accel*(self.duration-t)**2
        fraction = min(1.0, max(0.0, distance/self.distance)) if self.distance else 1.0
        position = tuple(a + fraction*d for a, d in zip(self.start, self.delta))
        yaw_step = min(abs(self.yaw_delta), self.yaw_rate*self.time)
        yaw = wrap_angle(self.yaw_start + math.copysign(yaw_step, self.yaw_delta))
        self.done = self.time >= self.duration and yaw_step >= abs(self.yaw_delta)
        self.command = (position, yaw)
        return self.command


class FinalPoseRefiner:
    def __init__(self, parameters, offset, tolerances, max_gap, pose_timeout):
        self.p = dict(parameters)
        validate_parameters(self.p)
        self.offset = tuple(offset)
        self.tolerances = tuple(tolerances)
        self.max_gap = max_gap
        self.pose_timeout = pose_timeout
        self.reason = ''
        self.stats = 'samples=0 span=0 std=n/a target=n/a'
        self.generation = 0
        self.reset('initial')

    def reset(self, reason):
        self.mode = 'ALIGN'
        self.target = None
        self.target_stamp = None
        self.transition = None
        self.hold_yaw = None
        self.window_start = None
        self.window_deadline = None
        self.last_tick = None
        self.clear_window(reason)

    def clear_window(self, reason='', epoch=float('inf')):
        self.pending = deque()
        self.samples = []
        self.epoch = epoch
        self.reason = reason

    def within(self, error):
        return all(abs(v) <= limit for v, limit in zip(error, self.tolerances))

    def marker(self, stamp, received, position_body, yaw_error):
        if stamp < self.epoch or received < self.epoch:
            return
        # 只接受源时间与接收时间均在本轮窗口内的观测：截止后先于控制
        # 定时器到达的图像不得进入最终均值。
        if self.window_deadline is not None and (
                stamp > self.window_deadline + 1e-9
                or received > self.window_deadline + 1e-9):
            return
        # 去重只对已入队的观测生效：乱序/未来时间戳不抬升水位，
        # 后续正常时间戳的观测仍可入队。
        queued = [s for s in self.pending if s[0] >= stamp]
        if any(s[0] == stamp for s in queued):
            return
        self.pending.append((stamp, received, tuple(position_body), yaw_error))
        # Bounded even with a stalled odometry producer / unusually high input rate.
        if len(self.pending) > 256:
            self.pending.clear()

    def process_pairs(self, now, buffer):
        """Pair queued Marker observations with interpolated local poses.

        确定无法配对的样本（odom 已越过其时间戳但插值失败的历史间隙）
        直接丢弃；尚待后续 VIO 覆盖的样本暂存队尾，不阻塞其后可配对的
        样本。截止时不等待未完成配对（deadline tick 后由 clear_window
        丢弃暂存项）。
        """
        remaining = deque()
        while self.pending:
            stamp, received, body, error = self.pending.popleft()
            if now - received > self.p['yaw_sample_pending_timeout_sec']:
                continue
            pose = buffer.lookup(stamp, self.max_gap)
            if pose is None:
                if buffer.newest_stamp is not None and stamp <= buffer.newest_stamp:
                    # 历史间隙：VIO 单调前进，该时间戳永远不会可配对。
                    continue
                # 等待未来的 VIO 样本：暂存但不阻塞后续处理。
                remaining.append((stamp, received, body, error))
                continue
            if stamp < self.window_start:
                # Marker 到达于窗口内，但配对的 odom 早于窗口入场：不可用。
                continue
            angle = wrap_angle(yaw_from_quaternion(pose.quaternion) + error)
            rotated = rotate_vector_by_quaternion(pose.quaternion, body)
            marker_local = tuple(a+b for a, b in zip(pose.position, rotated))
            if self.samples and stamp <= self.samples[-1][0]:
                continue  # 重复/乱序源时间戳不参与计算
            self.samples.append((stamp, angle, marker_local))
        self.pending = remaining

    def commit(self, now, local_pose):
        """Build the fixed target from all collected samples at deadline.

        The transition is created immediately so the committing control tick
        already emits a position setpoint.
        """
        if not self.samples:
            self.reset('empty_window')
            return
        angles = [s[1] for s in self.samples]
        mean = circular_mean(angles, self.hold_yaw)
        deviation = circular_std(angles)
        marker = tuple(
            sum(s[2][axis] for s in self.samples) / len(self.samples)
            for axis in range(3)
        )
        offset = rotate_vector_by_yaw(self.offset, mean)
        self.target = (tuple(a-b for a, b in zip(marker, offset)), mean)
        self.target_stamp = self.samples[-1][0]
        self.mode = 'POSE_ALIGNING'
        self.transition = PoseTransition(
            local_pose.position, self.hold_yaw, self.target, self.p
        )
        self.reason = 'target accepted'
        self.generation += 1
        self.stats = 'samples=%d span=%.3f std=%.2f target=(%.3f, %.3f, %.3f, %.2fdeg)' % (
            len(angles),
            self.samples[-1][0]-self.samples[0][0],
            math.degrees(deviation),
            self.target[0][0], self.target[0][1], self.target[0][2],
            math.degrees(mean),
        )
        self.clear_window('target accepted')

    def tick(self, now, dt, pose_fresh, local_pose, speed, error, buffer):
        """Advance the refiner one control period.

        ``error``/``speed``/``pose_fresh`` only gate ALIGN entry; once the
        window starts they no longer reset anything. The window deadline is
        evaluated here so it fires on the control timer even with no new
        images.
        """
        if self.last_tick is not None and now < self.last_tick:
            self.reset('clock reversal')
        self.last_tick = now
        if local_pose is None:
            self.reset('localization unavailable')
            return
        if self.mode == 'ALIGN':
            # 入场要求新鲜视觉与 XYZ 容差同时满足：视觉已超时但旧误差
            # 尚未清除时不得凭旧误差启动采样。
            if pose_fresh and error is not None and self.within(error):
                self.mode = 'YAW_SAMPLING'
                self.hold_yaw = yaw_from_quaternion(local_pose.quaternion)
                self.window_start = now
                self.window_deadline = (
                    now + self.p['yaw_sampling_duration_sec'])
                self.clear_window('sampling', epoch=now)
            return
        if self.mode == 'YAW_SAMPLING':
            # 每个控制周期都消费队列：已入队样本的配对只取决于自身
            # 时间戳、窗口范围和配对超时，不依赖当前视觉新鲜度。
            # 视觉新鲜度仅用于入场判断与速度输出（节点侧）。
            self.process_pairs(now, buffer)
            if now - self.window_start >= self.p['yaw_sampling_duration_sec'] - 1e-9:
                self.commit(now, local_pose)
            return
        if self.mode == 'POSE_ALIGNING':
            if self.transition is None:
                self.transition = PoseTransition(
                    local_pose.position, self.hold_yaw, self.target, self.p
                )
            else:
                self.transition.step(dt)
            if self.transition.done:
                self.mode = 'POS_HOLD'
                self.reason = 'transition complete'
            return
        # POS_HOLD: fixed target, nothing further to compute.

    def status(self):
        span = self.samples[-1][0]-self.samples[0][0] if self.samples else 0.0
        return 'refine=%s n=%d span=%.3f | %s | %s' % (
            self.mode, len(self.samples), span, self.stats, self.reason)
