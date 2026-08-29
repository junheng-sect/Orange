#!/usr/bin/env python3

import math
import unittest

from front_marker_control.control import BodyVelocityController, Vector3SlewRateLimiter
from front_marker_control.geometry import (
    marker_heading_body,
    normalize_quaternion,
    position_error_body,
)


class Quaternion(object):
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Position(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Pose(object):
    def __init__(self, x, y, z):
        self.position = Position(x, y, z)


def multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def body_marker_quaternion(yaw):
    yaw_rotation = (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
    front_facing = (0.5, -0.5, -0.5, 0.5)
    return Quaternion(*multiply(yaw_rotation, front_facing))


class GeometryTest(unittest.TestCase):
    def test_quaternion_normalization_and_invalid_input(self):
        normalized = normalize_quaternion(Quaternion(1.0, 0.0, 0.0, 1.0))
        self.assertTrue(math.isclose(sum(value * value for value in normalized), 1.0))
        with self.assertRaises(ValueError):
            normalize_quaternion(Quaternion(0.0, 0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            normalize_quaternion(Quaternion(float('nan'), 0.0, 0.0, 1.0))

    def test_marker_heading_preserves_body_yaw_sign(self):
        self.assertTrue(math.isclose(
            marker_heading_body(body_marker_quaternion(0.3)),
            0.3,
            abs_tol=1e-9,
        ))
        self.assertTrue(math.isclose(
            marker_heading_body(body_marker_quaternion(-0.3)),
            -0.3,
            abs_tol=1e-9,
        ))

    def test_position_error_is_observed_minus_target(self):
        error = position_error_body(Pose(1.0, -0.2, 0.1), (0.8, 0.0, 0.0))
        self.assertTrue(math.isclose(error[0], 0.2, abs_tol=1e-12))
        self.assertTrue(math.isclose(error[1], -0.2, abs_tol=1e-12))
        self.assertTrue(math.isclose(error[2], 0.1, abs_tol=1e-12))


class ControllerTest(unittest.TestCase):
    def controller(self, enable_yaw=True):
        return BodyVelocityController(
            0.5, 0.0, 0.03,
            0.6, 0.0, 0.08,
            0.4, 0.0, 0.0,
            0.2, 0.8, 0.8,
            0.03, 0.04, 0.06, 0.10,
            enable_yaw,
        )

    def test_limits_deadbands_and_yaw_switch(self):
        enabled = self.controller(True)
        velocity, yaw_rate, filtered = enabled.update((1.0, 2.0, 2.0), 0.3, 0.05)
        self.assertTrue(math.isclose(velocity[0], 0.2, abs_tol=1e-9))
        self.assertLessEqual(math.hypot(velocity[1], velocity[2]), 0.8 + 1e-9)
        self.assertTrue(math.isclose(filtered, 0.3, abs_tol=1e-9))
        self.assertTrue(math.isclose(yaw_rate, 0.12, abs_tol=1e-9))

        disabled = self.controller(False)
        _, yaw_rate, filtered = disabled.update((0.0, 0.0, 0.0), 0.3, 0.05)
        self.assertEqual(yaw_rate, 0.0)
        self.assertTrue(math.isclose(filtered, 0.3, abs_tol=1e-9))

    def test_reset_clears_derivative_and_filter_history(self):
        controller = self.controller(True)
        controller.update((0.5, 0.0, 0.0), 0.3, 0.05)
        controller.reset()
        velocity, _, filtered = controller.update((-0.5, 0.0, 0.0), -0.3, 0.05)
        self.assertTrue(math.isclose(velocity[0], -0.2, abs_tol=1e-9))
        self.assertTrue(math.isclose(filtered, -0.3, abs_tol=1e-9))


class Vector3SlewRateLimiterTest(unittest.TestCase):
    def test_limits_vector_norm_and_preserves_direction(self):
        limiter = Vector3SlewRateLimiter()
        value = limiter.update((0.3, 0.4, 0.0), 0.5, 0.1)
        self.assertTrue(math.isclose(value[0], 0.03, abs_tol=1e-12))
        self.assertTrue(math.isclose(value[1], 0.04, abs_tol=1e-12))
        self.assertTrue(math.isclose(math.hypot(value[0], value[1]), 0.05))

    def test_reaches_target_without_overshoot_and_returns_to_zero(self):
        limiter = Vector3SlewRateLimiter()
        self.assertEqual(limiter.update((0.02, 0.0, 0.0), 0.5, 0.1), (0.02, 0.0, 0.0))
        self.assertEqual(limiter.update((0.0, 0.0, 0.0), 1.0, 0.1), (0.0, 0.0, 0.0))

    def test_sign_reversal_and_reset(self):
        limiter = Vector3SlewRateLimiter()
        limiter.reset((0.2, 0.0, 0.0))
        value = limiter.update((-0.2, 0.0, 0.0), 0.5, 0.1)
        self.assertTrue(math.isclose(value[0], 0.15, abs_tol=1e-12))
        self.assertEqual(limiter.reset(), (0.0, 0.0, 0.0))

    def test_rejects_invalid_inputs(self):
        limiter = Vector3SlewRateLimiter()
        with self.assertRaises(ValueError):
            limiter.update((1.0, 2.0), 0.5, 0.1)
        with self.assertRaises(ValueError):
            limiter.update((float('nan'), 0.0, 0.0), 0.5, 0.1)
        with self.assertRaises(ValueError):
            limiter.update((0.0, 0.0, 0.0), 0.0, 0.1)
        with self.assertRaises(ValueError):
            limiter.update((0.0, 0.0, 0.0), 0.5, 0.0)


class MavrosMaskTest(unittest.TestCase):
    def test_velocity_only_mask(self):
        try:
            from mavros_msgs.msg import PositionTarget
            from front_marker_control.mavros_interface import velocity_only_type_mask
        except ImportError:
            self.skipTest('mavros_msgs is unavailable')
        mask = velocity_only_type_mask()
        self.assertFalse(mask & PositionTarget.IGNORE_VX)
        self.assertFalse(mask & PositionTarget.IGNORE_VY)
        self.assertFalse(mask & PositionTarget.IGNORE_VZ)
        self.assertFalse(mask & PositionTarget.IGNORE_YAW_RATE)
        self.assertTrue(mask & PositionTarget.IGNORE_PX)
        self.assertTrue(mask & PositionTarget.IGNORE_YAW)


if __name__ == '__main__':
    unittest.main()
