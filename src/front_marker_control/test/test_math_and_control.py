#!/usr/bin/env python3

import math
import unittest

from front_marker_control.control import (
    BodyVelocityController,
    PIDController,
    Vector3SlewRateLimiter,
)
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
    def controller(self, enable_yaw=True, **overrides):
        parameters = {
            'kp_x': 0.5,
            'ki_x': 0.0,
            'kd_x': 0.03,
            'kp_yz': 0.6,
            'ki_yz': 0.0,
            'kd_yz': 0.08,
            'kp_yaw': 0.4,
            'ki_yaw': 0.0,
            'kd_yaw': 0.0,
            'vx_limit': 0.2,
            'v_limit': 0.8,
            'yaw_rate_limit': 0.8,
            'velocity_deadband': 0.03,
            'z_error_deadband': 0.04,
            'yaw_error_deadband': 0.06,
            'yaw_lpf_tau': 0.10,
            'enable_yaw_control': enable_yaw,
            'integral_limit_x': 1.0,
            'integral_limit_yz': 1.0,
            'integral_limit_yaw': 1.0,
        }
        parameters.update(overrides)
        return BodyVelocityController(**parameters)

    def test_pid_integral_limit_is_configurable_and_validated(self):
        pid = PIDController(0.0, 1.0, 0.0, integral_limit=0.25)
        self.assertTrue(math.isclose(pid.update(1.0, 1.0), 0.25))
        self.assertTrue(math.isclose(pid.integral, 0.25))

        disabled_integral = PIDController(0.0, 1.0, 0.0, integral_limit=0.0)
        self.assertEqual(disabled_integral.update(1.0, 1.0), 0.0)
        self.assertEqual(disabled_integral.integral, 0.0)

        for invalid_limit in (-0.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                PIDController(0.0, 1.0, 0.0, integral_limit=invalid_limit)

    def test_pid_freezes_integral_when_scalar_output_saturates(self):
        pid = PIDController(
            0.0, 1.0, 0.0, output_limit=0.5, integral_limit=2.0
        )
        self.assertTrue(math.isclose(pid.update(1.0, 0.2), 0.2))
        self.assertTrue(math.isclose(pid.update(1.0, 0.2), 0.4))
        self.assertTrue(math.isclose(pid.update(1.0, 0.2), 0.5))
        self.assertTrue(math.isclose(pid.integral, 0.4))
        self.assertTrue(math.isclose(pid.update(1.0, 0.2), 0.5))
        self.assertTrue(math.isclose(pid.integral, 0.4))

    def test_pid_opposing_error_unwinds_saturated_integral(self):
        pid = PIDController(
            0.0, 1.0, 0.0, output_limit=2.0, integral_limit=2.0
        )
        self.assertTrue(math.isclose(pid.update(1.0, 1.0), 1.0))
        pid.output_limit = 0.5

        self.assertTrue(math.isclose(pid.update(-1.0, 0.2), 0.5))
        self.assertTrue(math.isclose(pid.integral, 0.8))
        self.assertTrue(math.isclose(pid.update(-1.0, 0.2), 0.5))
        self.assertTrue(math.isclose(pid.integral, 0.6))
        self.assertTrue(math.isclose(pid.update(-1.0, 0.2), 0.4))
        self.assertTrue(math.isclose(pid.integral, 0.4))

    def test_grouped_integral_limits_reach_each_pid(self):
        controller = self.controller(
            integral_limit_x=0.1,
            integral_limit_yz=0.2,
            integral_limit_yaw=0.3,
        )
        self.assertTrue(math.isclose(controller.pid_x.integral_limit, 0.1))
        self.assertTrue(math.isclose(controller.pid_y.integral_limit, 0.2))
        self.assertTrue(math.isclose(controller.pid_z.integral_limit, 0.2))
        self.assertTrue(math.isclose(controller.pid_yaw.integral_limit, 0.3))

    def test_yz_vector_saturation_freezes_same_direction_integrals(self):
        controller = self.controller(
            kp_yz=1.0,
            ki_yz=1.0,
            kd_yz=0.0,
            v_limit=0.5,
            velocity_deadband=0.0,
            z_error_deadband=0.0,
            integral_limit_yz=2.0,
        )

        for _ in range(2):
            velocity, _, _ = controller.update((0.0, 0.5, 0.5), 0.0, 0.1)
            self.assertTrue(
                math.isclose(math.hypot(velocity[1], velocity[2]), 0.5)
            )
            self.assertTrue(math.isclose(velocity[1], velocity[2]))
            self.assertEqual(controller.pid_y.integral, 0.0)
            self.assertEqual(controller.pid_z.integral, 0.0)

    def test_yz_vector_saturation_allows_per_axis_unwinding(self):
        controller = self.controller(
            kp_yz=0.0,
            ki_yz=1.0,
            kd_yz=0.0,
            v_limit=0.5,
            velocity_deadband=0.0,
            z_error_deadband=0.0,
            integral_limit_yz=2.0,
        )
        controller.pid_y.integral = 0.8
        controller.pid_z.integral = 0.8

        velocity, _, _ = controller.update((0.0, -1.0, 1.0), 0.0, 0.1)
        self.assertTrue(math.isclose(math.hypot(velocity[1], velocity[2]), 0.5))
        self.assertTrue(math.isclose(velocity[1] / velocity[2], 0.7 / 0.9))
        self.assertTrue(math.isclose(controller.pid_y.integral, 0.7))
        self.assertTrue(math.isclose(controller.pid_z.integral, 0.8))

    def test_output_deadband_does_not_freeze_unsaturated_integral(self):
        controller = self.controller(
            kp_yz=0.0,
            ki_yz=1.0,
            kd_yz=0.0,
            v_limit=1.0,
            velocity_deadband=0.2,
            z_error_deadband=0.0,
            integral_limit_yz=1.0,
        )
        velocity, _, _ = controller.update((0.0, 0.1, 0.0), 0.0, 1.0)
        self.assertEqual(velocity[1], 0.0)
        self.assertTrue(math.isclose(controller.pid_y.integral, 0.1))

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
