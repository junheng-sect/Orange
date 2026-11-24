#!/usr/bin/env python3
"""Units tests for odometry caching, interpolation and hold-target geometry."""

import math
import unittest

from front_marker_control.control import wrap_angle
from front_marker_control.geometry import (
    normalize_quaternion,
    position_hold_target,
    rotate_vector_by_quaternion,
    rotate_vector_by_yaw,
    yaw_from_quaternion,
)
from front_marker_control.localization import (
    LocalPoseBuffer,
    interpolate_local_pose,
    slerp_quaternion,
    yaw_pose,
    pose_yaw,
)


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def rotate_z(vector, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        vector[0] * cosine - vector[1] * sine,
        vector[0] * sine + vector[1] * cosine,
        vector[2],
    )


class QuaternionRotationTest(unittest.TestCase):
    def test_yaw_extraction_and_quaternion_rotation(self):
        for yaw in (0.0, 0.3, -1.2, 2.9):
            quaternion = yaw_quaternion(yaw)
            self.assertTrue(math.isclose(
                yaw_from_quaternion(quaternion), wrap_angle(yaw), abs_tol=1e-9,
            ))
            rotated = rotate_vector_by_quaternion(quaternion, (1.0, 0.0, 0.2))
            expected = rotate_z((1.0, 0.0, 0.2), yaw)
            for actual, want in zip(rotated, expected):
                self.assertTrue(math.isclose(actual, want, abs_tol=1e-9))

    def test_yaw_rotation_matches_quaternion_rotation(self):
        yaw = 0.7
        vector = (0.5, -0.3, 0.1)
        by_yaw = rotate_vector_by_yaw(vector, yaw)
        by_quaternion = rotate_vector_by_quaternion(yaw_quaternion(yaw), vector)
        for actual, want in zip(by_quaternion, by_yaw):
            self.assertTrue(math.isclose(actual, want, abs_tol=1e-9))

    def test_normalize_quaternion_output_is_unit(self):
        normalized = normalize_quaternion(
            type('Q', (), {'x': 2.0, 'y': 0.0, 'z': 0.0, 'w': 0.0})()
        )
        self.assertTrue(math.isclose(math.fsum(
            value * value for value in normalized
        ), 1.0, abs_tol=1e-12))


class PositionHoldTargetTest(unittest.TestCase):
    def test_identity_body_yields_marker_minus_offset(self):
        p_target, yaw_target = position_hold_target(
            (2.0, 1.0, 0.5),
            yaw_quaternion(0.0),
            (0.8, 0.0, 0.0),
            0.0,
            (0.8, 0.0, 0.0),
        )
        # p_marker = p_body + (0.8, 0, 0); p_target = p_marker - (0.8, 0, 0)
        for actual, want in zip(p_target, (2.0, 1.0, 0.5)):
            self.assertTrue(math.isclose(actual, want, abs_tol=1e-9))
        self.assertTrue(math.isclose(yaw_target, 0.0, abs_tol=1e-9))

    def test_nonzero_local_yaw_and_offset_geometry(self):
        # p_target = p_marker - Rz(yaw_target) * target_xyz_body with zero
        # marker yaw error, so the hold offset rotates by the local yaw.
        local_yaw = 0.4
        p_body = (1.0, 2.0, 3.0)
        p_marker_body = (0.5, -0.25, 0.1)
        target_xyz_body = (0.8, 0.0, 0.0)
        p_target, yaw_target = position_hold_target(
            p_body,
            yaw_quaternion(local_yaw),
            p_marker_body,
            0.0,
            target_xyz_body,
        )
        marker_local = rotate_vector_by_quaternion(
            yaw_quaternion(local_yaw), p_marker_body
        )
        hold_offset_local = rotate_z(target_xyz_body, local_yaw)
        for index in range(3):
            expected = (
                p_body[index] + marker_local[index] - hold_offset_local[index]
            )
            self.assertTrue(math.isclose(
                p_target[index], expected, abs_tol=1e-9,
            ))
        self.assertTrue(math.isclose(yaw_target, local_yaw, abs_tol=1e-9))

    def test_marker_yaw_error_is_added_and_wrapped(self):
        local_yaw = 3.0
        marker_yaw_error = 0.4  # wraps to ~ -2.883
        _, yaw_target = position_hold_target(
            (0.0, 0.0, 0.0),
            yaw_quaternion(local_yaw),
            (1.0, 0.0, 0.0),
            marker_yaw_error,
            (0.8, 0.0, 0.0),
        )
        expected = wrap_angle(local_yaw + marker_yaw_error)
        self.assertTrue(math.isclose(yaw_target, expected, abs_tol=1e-9))
        self.assertLess(yaw_target, math.pi)

        _, no_yaw = position_hold_target(
            (0.0, 0.0, 0.0),
            yaw_quaternion(local_yaw),
            (1.0, 0.0, 0.0),
            marker_yaw_error,
            (0.8, 0.0, 0.0),
            apply_yaw_correction=False,
        )
        self.assertTrue(math.isclose(no_yaw, local_yaw, abs_tol=1e-9))

    def test_nonzero_offset_uses_target_yaw_rotation(self):
        # p_target = p_marker - Rz(yaw_target) * target_xyz_body
        local_yaw = 0.2
        marker_yaw_error = 0.3
        p_body = (1.0, -0.5, 0.4)
        p_marker_body = (0.9, 0.1, -0.1)
        target_xyz_body = (0.8, 0.05, -0.05)
        p_target, yaw_target = position_hold_target(
            p_body, yaw_quaternion(local_yaw), p_marker_body,
            marker_yaw_error, target_xyz_body,
        )
        yaw_expected = wrap_angle(local_yaw + marker_yaw_error)
        p_marker_local = tuple(
            p_body[index]
            + rotate_vector_by_quaternion(
                yaw_quaternion(local_yaw), p_marker_body
            )[index]
            for index in range(3)
        )
        hold_offset_local = rotate_z(target_xyz_body, yaw_expected)
        for index in range(3):
            expected = p_marker_local[index] - hold_offset_local[index]
            self.assertTrue(math.isclose(
                p_target[index], expected, abs_tol=1e-9,
            ))

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            position_hold_target(
                (float('nan'), 0.0, 0.0), yaw_quaternion(0.0),
                (1.0, 0.0, 0.0), 0.0, (0.8, 0.0, 0.0),
            )
        with self.assertRaises(ValueError):
            position_hold_target(
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0), 0.0, (0.8, 0.0, 0.0),
            )
        with self.assertRaises(ValueError):
            position_hold_target(
                (0.0, 0.0, 0.0), yaw_quaternion(0.0),
                (1.0, 0.0, 0.0), float('nan'), (0.8, 0.0, 0.0),
            )


class SlerpInterpolationTest(unittest.TestCase):
    def test_midpoint_slerp_yaw(self):
        start = yaw_quaternion(0.0)
        end = yaw_quaternion(1.0)
        middle = slerp_quaternion(start, end, 0.5)
        self.assertTrue(math.isclose(
            yaw_from_quaternion(middle), 0.5, abs_tol=1e-9,
        ))

    def test_slerp_takes_shortest_path_across_sign_flip(self):
        start = yaw_quaternion(3.1)
        end = tuple(-value for value in yaw_quaternion(-3.1))
        middle = slerp_quaternion(start, end, 0.5)
        # Near pi: the shortest path passes through pi, not through 0.
        self.assertTrue(math.isclose(
            abs(yaw_from_quaternion(middle)), math.pi, abs_tol=0.01,
        ))

    def test_interpolate_pose_midpoint(self):
        first = yaw_pose(0.0, (0.0, 0.0, 0.0), 0.0)
        second = yaw_pose(1.0, (1.0, 2.0, 4.0), 0.5)
        blended = interpolate_local_pose(first, second, 0.5)
        for actual, want in zip(blended.position, (0.5, 1.0, 2.0)):
            self.assertTrue(math.isclose(actual, want, abs_tol=1e-9))
        self.assertTrue(math.isclose(pose_yaw(blended), 0.25, abs_tol=1e-9))

    def test_interpolate_rejects_out_of_range_and_bad_gap(self):
        first = yaw_pose(0.0, (0.0, 0.0, 0.0), 0.0)
        second = yaw_pose(1.0, (1.0, 0.0, 0.0), 0.0)
        with self.assertRaises(ValueError):
            interpolate_local_pose(first, second, 1.5)
        with self.assertRaises(ValueError):
            interpolate_local_pose(first, second, -0.1)
        reversed_pair = yaw_pose(1.0, (0.0, 0.0, 0.0), 0.0)
        with self.assertRaises(ValueError):
            interpolate_local_pose(reversed_pair, first, 0.5)


class LocalPoseBufferTest(unittest.TestCase):
    def buffer_with_samples(self):
        buffer = LocalPoseBuffer()
        for stamp in (1.0, 1.05, 1.10, 1.15):
            self.assertIsNone(buffer.accept(
                stamp, 'odom',
                (stamp, 0.0, 0.0), yaw_quaternion(stamp),
            ))
        return buffer

    def test_lookup_exact_and_interpolated(self):
        buffer = self.buffer_with_samples()
        exact = buffer.lookup(1.10, 0.10)
        self.assertEqual(exact.stamp, 1.10)
        middle = buffer.lookup(1.125, 0.10)
        self.assertTrue(math.isclose(middle.position[0], 1.125, abs_tol=1e-9))
        self.assertTrue(math.isclose(pose_yaw(middle), 1.125, abs_tol=1e-6))

    def test_lookup_rejects_gap_above_max(self):
        buffer = LocalPoseBuffer()
        buffer.accept(1.0, 'odom', (0.0, 0.0, 0.0), yaw_quaternion(0.0))
        buffer.accept(1.2, 'odom', (0.2, 0.0, 0.0), yaw_quaternion(0.2))
        # gap 0.2 > 0.1: no interpolation across the hole.
        self.assertIsNone(buffer.lookup(1.1, 0.10))
        # Endpoints themselves remain usable.
        self.assertIsNotNone(buffer.lookup(1.0, 0.10))

    def test_lookup_never_extrapolates(self):
        buffer = self.buffer_with_samples()
        self.assertIsNone(buffer.lookup(0.5, 0.10))
        self.assertIsNone(buffer.lookup(2.0, 0.10))

    def test_invalid_values_clear_cache(self):
        buffer = self.buffer_with_samples()
        reason = buffer.accept(
            1.2, 'odom', (float('nan'), 0.0, 0.0), yaw_quaternion(0.0)
        )
        self.assertIsNotNone(reason)
        self.assertEqual(len(buffer), 0)
        # Old cached poses are gone: lookups fail afterwards.
        self.assertIsNone(buffer.lookup(1.10, 0.10))

    def test_frame_change_clears_cache_and_reports(self):
        buffer = self.buffer_with_samples()
        reason = buffer.accept(
            1.2, 'other_frame', (0.0, 0.0, 0.0), yaw_quaternion(0.0)
        )
        self.assertIn('坐标系变化', reason)
        self.assertEqual(len(buffer), 0)
        # The new frame starts a fresh cache.
        self.assertIsNone(buffer.accept(
            1.3, 'other_frame', (0.0, 0.0, 0.0), yaw_quaternion(0.0)
        ))
        self.assertEqual(len(buffer), 1)

    def test_non_increasing_stamps_are_dropped(self):
        buffer = self.buffer_with_samples()
        self.assertIsNotNone(buffer.accept(
            1.15, 'odom', (0.0, 0.0, 0.0), yaw_quaternion(0.0)
        ))
        self.assertIsNotNone(buffer.accept(
            1.0, 'odom', (0.0, 0.0, 0.0), yaw_quaternion(0.0)
        ))
        self.assertEqual(len(buffer), 4)

    def test_invalid_stamp_and_frame_are_rejected(self):
        buffer = LocalPoseBuffer()
        # Non-finite stamp is rejected without clearing (nothing cached yet).
        self.assertIsNotNone(buffer.accept(
            float('nan'), 'odom', (0.0, 0.0, 0.0), yaw_quaternion(0.0)
        ))
        # Empty frame is rejected.
        self.assertIsNotNone(buffer.accept(
            1.0, '', (0.0, 0.0, 0.0), yaw_quaternion(0.0)
        ))
        # Zero stamp: finite but non-monotonic, dropped by the stamp rule.
        buffer.accept(0.5, 'odom', (0.0, 0.0, 0.0), yaw_quaternion(0.0))
        self.assertIsNotNone(buffer.accept(
            0.5, 'odom', (0.0, 0.0, 0.0), yaw_quaternion(0.0)
        ))
        self.assertEqual(len(buffer), 1)

    def test_capacity_is_bounded(self):
        buffer = LocalPoseBuffer(capacity=10)
        for index in range(30):
            buffer.accept(float(index), 'odom', (0.0, 0.0, 0.0), (0, 0, 0, 1))
        self.assertEqual(len(buffer), 10)
        self.assertEqual(buffer.newest_stamp, 29.0)


class PositionOnlyMaskTest(unittest.TestCase):
    def test_position_only_mask(self):
        try:
            from mavros_msgs.msg import PositionTarget
            from front_marker_control.mavros_interface import (
                position_only_type_mask,
            )
        except ImportError:
            self.skipTest('mavros_msgs is unavailable')
        mask = position_only_type_mask()
        self.assertFalse(mask & PositionTarget.IGNORE_PX)
        self.assertFalse(mask & PositionTarget.IGNORE_PY)
        self.assertFalse(mask & PositionTarget.IGNORE_PZ)
        self.assertFalse(mask & PositionTarget.IGNORE_YAW)
        self.assertTrue(mask & PositionTarget.IGNORE_VX)
        self.assertTrue(mask & PositionTarget.IGNORE_VY)
        self.assertTrue(mask & PositionTarget.IGNORE_VZ)
        self.assertTrue(mask & PositionTarget.IGNORE_YAW_RATE)
        self.assertTrue(mask & PositionTarget.IGNORE_AFX)
        self.assertTrue(mask & PositionTarget.IGNORE_AFY)
        self.assertTrue(mask & PositionTarget.IGNORE_AFZ)


if __name__ == '__main__':
    unittest.main()
