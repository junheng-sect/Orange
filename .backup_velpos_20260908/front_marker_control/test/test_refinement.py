#!/usr/bin/env python3
"""Deterministic source-time, geometry, state-machine and transition tests."""
import math
import unittest

from front_marker_control.refinement import DEFAULTS, FinalPoseRefiner, PoseTransition, robust_angles, validate_parameters
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
        for _ in range(32):
            pose = self.pose(self.now)
            self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
            self.now += 1/30
        self.assertEqual(self.engine.mode, 'YAW_SAMPLING')

    def sample(self, yaw=0, body=(0.8, 0, 0), position=(0, 0, 0), body_yaw=0):
        pose = self.pose(self.now, position, body_yaw)
        self.engine.marker(self.now, self.now, body, yaw)
        self.engine.tick(self.now, 1/30, True, pose, 0, (0, 0, 0), self.buffer)
        self.now += 1/30

    def target(self, yaw=0):
        self.start()
        for _ in range(92):
            self.sample(yaw)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')

    def test_wrap_mean_and_outlier(self):
        values = radians([179, -179]*40 + [90]*10)
        mean, std, indices, reason = robust_angles(values, self.parameters)
        self.assertFalse(reason)
        self.assertAlmostEqual(abs(mean), math.pi)
        self.assertLess(std, math.radians(1.1))
        self.assertEqual(len(indices), 80)

    def test_quality_gates(self):
        for values, expected in (([0]*59, 'insufficient'),
                                 ([0]*70+[40]*30, 'ratio'),
                                 ([-7, 7]*45, 'std')):
            self.assertIn(expected, robust_angles(radians(values), self.parameters)[3])

    def test_invalid_parameters(self):
        for key, value in (('yaw_sampling_min_samples', 2.1),
                           ('yaw_sampling_duration_sec', 0),
                           ('yaw_sampling_min_inlier_ratio', 1.1),
                           ('pose_transition_speed_mps', float('nan'))):
            p = dict(self.parameters, **{key: value})
            with self.assertRaises(ValueError):
                validate_parameters(p)

    def test_single_frame_never_locks_and_duplicates_do_not_count(self):
        self.start()
        self.sample()
        stamp = self.now-1/30
        for _ in range(90):
            self.engine.marker(stamp, self.now, (0.8, 0, 0), 0)
        self.assertEqual(len(self.engine.samples), 1)
        self.assertEqual(len(self.engine.pending), 0)
        self.assertIsNone(self.engine.target)

    def test_source_interpolation_waits_and_has_no_receive_fallback(self):
        self.start()
        stamp = self.now
        self.engine.marker(stamp, stamp+0.02, (0.8, 0, 0), 0)
        self.engine.process_pairs(stamp+0.02, self.buffer)
        self.assertEqual(len(self.engine.samples), 0)
        self.pose(stamp+0.03)
        self.engine.process_pairs(stamp+0.04, self.buffer)
        self.assertEqual(len(self.engine.samples), 1)
        self.assertEqual(self.engine.samples[0][0], stamp)

    def test_pair_timeout_and_gap_restart(self):
        self.start()
        self.engine.marker(self.now, self.now, (0.8, 0, 0), 0)
        self.engine.process_pairs(self.now+0.51, self.buffer)
        self.assertEqual(self.engine.mode, 'ALIGN')
        self.assertFalse(self.engine.samples)
        self.assertIsNone(self.engine.stable_since)

    def test_gap_does_not_join_two_short_windows(self):
        self.start()
        for _ in range(30):
            self.sample()
        self.now += 0.11
        self.sample()
        self.assertEqual(self.engine.mode, 'ALIGN')
        self.assertFalse(self.engine.samples)

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
        # A synthetic moving observation proves we average world targets, not body errors.
        for i in range(92):
            yaw = math.radians(i/20)
            position = (i/1000, 0, 0)
            body = ((.8-position[0])*math.cos(yaw), -(.8-position[0])*math.sin(yaw), 0)
            self.sample(math.radians(8)-yaw, body, position, yaw)
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertAlmostEqual(self.engine.target[1], math.radians(8))
        self.assertAlmostEqual(self.engine.target[0][1], -.8*math.sin(math.radians(8)))

    def test_correction_rejection(self):
        self.start()
        for _ in range(92):
            self.sample(math.radians(21))
        self.assertEqual(self.engine.mode, 'ALIGN')
        self.assertIn('correction', self.engine.reason)
        self.assertIsNone(self.engine.target)

    def test_speed_and_xyz_interrupt(self):
        for speed, error in ((.06, (0, 0, 0)), (0, (.13, 0, 0))):
            self.setUp()
            self.start()
            pose = self.pose(self.now)
            self.engine.tick(self.now, .03, True, pose, speed, error, self.buffer)
            self.assertEqual(self.engine.mode, 'ALIGN')
            self.assertIsNone(self.engine.stable_since)

    def test_localization_and_vision_reset_sampling(self):
        for local, fresh in ((None, True), (yaw_pose(100, (0, 0, 0), 0), False)):
            self.setUp()
            self.start()
            self.sample()
            self.engine.tick(self.now, .03, fresh, local, 0, (0, 0, 0), self.buffer)
            self.assertEqual(self.engine.mode, 'ALIGN')
            self.assertFalse(self.engine.samples)

    def test_independent_verification_and_resampling(self):
        self.target()
        for _ in range(32):
            self.sample()
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        for _ in range(94):
            self.sample()
        self.assertEqual(self.engine.mode, 'POS_HOLD')
        for _ in range(100):
            self.sample(math.radians(8))
        self.assertEqual(self.engine.mode, 'ALIGN')
        self.assertIn('residual', self.engine.reason)

    def test_visual_loss_never_verifies(self):
        self.target()
        for _ in range(200):
            pose = self.pose(self.now)
            self.engine.tick(self.now, 1/30, False, pose, 0, None, self.buffer)
            self.now += 1/30
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertIsNone(self.engine.stable_since)
        self.assertFalse(self.engine.samples)

    def test_noisy_verification_is_inconclusive(self):
        self.target()
        for i in range(180):
            self.sample(math.radians(8 if i%2 else -8))
        self.assertEqual(self.engine.mode, 'POSE_ALIGNING')
        self.assertIsNotNone(self.engine.target)

    def test_actual_tolerances_follow_target_heading(self):
        self.engine.target = ((0, 0, 0), math.pi/2)
        # Local Y is distance at a 90-degree heading; local X is lateral.
        self.assertTrue(self.engine.actual_pose_within(yaw_pose(10, (0, .10, 0), math.pi/2), 0))
        self.assertFalse(self.engine.actual_pose_within(yaw_pose(10, (.10, 0, 0), math.pi/2), 0))

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
