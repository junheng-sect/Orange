#!/usr/bin/env python3

import math
import unittest

from front_marker_control.control import (
    BodyVelocityController,
    FirstOrderLowPass,
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


class FirstOrderLowPassTest(unittest.TestCase):
    def test_half_step_when_dt_equals_tau_and_zero_tau_bypasses(self):
        filtered = FirstOrderLowPass(0.10)
        self.assertEqual(filtered.update(1.0, 0.10), 1.0)
        self.assertTrue(math.isclose(filtered.update(0.0, 0.10), 0.5))

        bypassed = FirstOrderLowPass(0.0)
        self.assertEqual(bypassed.update(1.0, 0.10), 1.0)
        self.assertEqual(bypassed.update(-1.0, 0.10), -1.0)


class ControllerTest(unittest.TestCase):
    def controller(self, enable_yaw=True, **overrides):
        parameters = {
            'kp_x': 0.5,
            'kp_y': 0.6,
            'kp_z': 0.6,
            'kp_yaw': 0.4,
            'vx_limit': 0.2,
            'vyz_limit': 0.8,
            'yaw_rate_limit': 0.8,
            'velocity_deadband_enter': 0.02,
            'velocity_deadband': 0.03,
            'z_error_deadband': 0.04,
            'yaw_error_deadband': 0.06,
            'yaw_lpf_tau': 0.20,
            'position_lpf_tau_x': 0.05,
            'position_lpf_tau_y': 0.10,
            'position_lpf_tau_z': 0.10,
            'enable_yaw_control': enable_yaw,
        }
        parameters.update(overrides)
        return BodyVelocityController(**parameters)

    def test_proportional_mapping_and_x_limit(self):
        controller = self.controller(
            position_lpf_tau_x=0.0,
            position_lpf_tau_y=0.0,
            position_lpf_tau_z=0.0,
        )
        velocity, _, _ = controller.update((0.1, 0.0, 0.0), 0.0, 0.05)
        self.assertTrue(math.isclose(velocity[0], 0.05, abs_tol=1e-12))

        saturated, _, _ = controller.update((1.0, 0.0, 0.0), 0.0, 0.05)
        self.assertTrue(math.isclose(saturated[0], 0.2, abs_tol=1e-12))
        negative, _, _ = controller.update((-1.0, 0.0, 0.0), 0.0, 0.05)
        self.assertTrue(math.isclose(negative[0], -0.2, abs_tol=1e-12))

    def test_proportional_mapping_uses_independent_xyz_gains(self):
        controller = self.controller(
            kp_x=2.0,
            kp_y=3.0,
            kp_z=4.0,
            vx_limit=1.0,
            vyz_limit=2.0,
            velocity_deadband_enter=0.0,
            velocity_deadband=0.0,
            z_error_deadband=0.0,
            position_lpf_tau_x=0.0,
            position_lpf_tau_y=0.0,
            position_lpf_tau_z=0.0,
        )
        velocity, _, _ = controller.update((0.1, 0.2, 0.3), 0.0, 0.05)
        for actual, expected in zip(velocity, (0.2, 0.6, 1.2)):
            self.assertTrue(math.isclose(actual, expected, abs_tol=1e-12))

    def test_proportional_gains_are_validated(self):
        for name in ('kp_x', 'kp_y', 'kp_z'):
            for invalid in (-0.1, float('nan'), float('inf')):
                with self.subTest(name=name, invalid=invalid):
                    with self.assertRaises(ValueError):
                        self.controller(**{name: invalid})
        with self.assertRaises(ValueError):
            self.controller(kp_yaw=float('inf'))

    def test_yz_combined_limit_preserves_direction(self):
        controller = self.controller(
            kp_y=1.0,
            kp_z=1.0,
            vyz_limit=0.5,
            velocity_deadband_enter=0.0,
            velocity_deadband=0.0,
            z_error_deadband=0.0,
            position_lpf_tau_y=0.0,
            position_lpf_tau_z=0.0,
        )

        for _ in range(2):
            velocity, _, _ = controller.update((0.0, 0.5, 0.5), 0.0, 0.1)
            self.assertTrue(
                math.isclose(math.hypot(velocity[1], velocity[2]), 0.5)
            )
            self.assertTrue(math.isclose(velocity[1], velocity[2]))

    def test_z_error_deadband_holds_and_releases(self):
        controller = self.controller(
            kp_y=1.0,
            kp_z=1.0,
            vx_limit=1.0,
            vyz_limit=1.0,
            velocity_deadband_enter=0.0,
            velocity_deadband=0.0,
            position_lpf_tau_y=0.0,
            position_lpf_tau_z=0.0,
        )
        held, _, _ = controller.update((0.0, 0.0, 0.03), 0.0, 0.1)
        self.assertEqual(held[2], 0.0)

        boundary, _, _ = controller.update((0.0, 0.0, 0.04), 0.0, 0.1)
        self.assertTrue(math.isclose(boundary[2], 0.04, abs_tol=1e-12))

        released, _, _ = controller.update((0.0, 0.0, 0.05), 0.0, 0.1)
        self.assertTrue(math.isclose(released[2], 0.05, abs_tol=1e-12))

    def test_yaw_error_deadband_holds_and_releases(self):
        controller = self.controller(yaw_lpf_tau=0.0, yaw_error_deadband=0.06)
        _, held, filtered = controller.update((0.0, 0.0, 0.0), 0.05, 0.05)
        self.assertEqual(held, 0.0)
        self.assertTrue(math.isclose(filtered, 0.05, abs_tol=1e-12))

        _, released, filtered = controller.update((0.0, 0.0, 0.0), 0.10, 0.05)
        self.assertTrue(math.isclose(released, 0.04, abs_tol=1e-12))
        self.assertTrue(math.isclose(filtered, 0.10, abs_tol=1e-12))

    def test_yaw_low_pass_filters_and_reset_clears_history(self):
        controller = self.controller(
            kp_yaw=0.4,
            yaw_rate_limit=1.0,
            yaw_error_deadband=0.0,
            yaw_lpf_tau=0.10,
        )
        _, yaw_rate, filtered = controller.update((0.0, 0.0, 0.0), 0.3, 0.1)
        self.assertTrue(math.isclose(filtered, 0.3, abs_tol=1e-9))
        self.assertTrue(math.isclose(yaw_rate, 0.12, abs_tol=1e-9))

        # dt = tau -> alpha = 0.5, so the filter moves halfway to zero.
        _, yaw_rate, filtered = controller.update((0.0, 0.0, 0.0), 0.0, 0.1)
        self.assertTrue(math.isclose(filtered, 0.15, abs_tol=1e-9))
        self.assertTrue(math.isclose(yaw_rate, 0.06, abs_tol=1e-9))

        controller.reset()
        _, _, filtered = controller.update((0.0, 0.0, 0.0), -0.3, 0.1)
        self.assertTrue(math.isclose(filtered, -0.3, abs_tol=1e-9))

    def test_output_deadband_hysteresis_prevents_boundary_chatter(self):
        controller = self.controller(
            kp_x=1.0,
            vx_limit=1.0,
            velocity_deadband_enter=0.02,
            velocity_deadband=0.03,
            position_lpf_tau_x=0.0,
        )
        values = [0.0, 0.031, 0.029, 0.021, 0.020, 0.029, 0.031]
        outputs = [
            controller.update((value, 0.0, 0.0), 0.0, 0.1)[0][0]
            for value in values
        ]
        self.assertEqual(
            outputs,
            [0.0, 0.031, 0.029, 0.021, 0.0, 0.0, 0.031],
        )

        controller.reset()
        negative_outputs = [
            controller.update((value, 0.0, 0.0), 0.0, 0.1)[0][0]
            for value in (-0.031, -0.029, -0.020, -0.029, -0.031)
        ]
        self.assertEqual(
            negative_outputs,
            [-0.031, -0.029, 0.0, 0.0, -0.031],
        )

    def test_output_deadband_axes_are_independent_and_resettable(self):
        controller = self.controller(
            kp_x=1.0,
            kp_y=1.0,
            kp_z=1.0,
            vx_limit=1.0,
            vyz_limit=1.0,
            velocity_deadband_enter=0.02,
            velocity_deadband=0.03,
            z_error_deadband=0.0,
            position_lpf_tau_x=0.0,
            position_lpf_tau_y=0.0,
            position_lpf_tau_z=0.0,
        )
        first, _, _ = controller.update((0.031, 0.010, 0.010), 0.0, 0.1)
        second, _, _ = controller.update((0.025, 0.031, 0.015), 0.0, 0.1)
        third, _, _ = controller.update((0.019, 0.025, 0.031), 0.0, 0.1)
        self.assertEqual(first, (0.031, 0.0, 0.0))
        self.assertEqual(second, (0.025, 0.031, 0.0))
        self.assertEqual(third, (0.0, 0.025, 0.031))

        controller.reset()
        after_reset, _, _ = controller.update((0.025, 0.025, 0.025), 0.0, 0.1)
        self.assertEqual(after_reset, (0.0, 0.0, 0.0))

    def test_output_deadband_accepts_zero_and_rejects_invalid_thresholds(self):
        controller = self.controller(
            kp_x=1.0,
            kp_y=1.0,
            kp_z=1.0,
            vx_limit=1.0,
            vyz_limit=1.0,
            velocity_deadband_enter=0.0,
            velocity_deadband=0.0,
            z_error_deadband=0.0,
            position_lpf_tau_x=0.0,
            position_lpf_tau_y=0.0,
            position_lpf_tau_z=0.0,
        )
        velocity, _, _ = controller.update((0.001, -0.001, 0.001), 0.0, 0.1)
        self.assertEqual(velocity, (0.001, -0.001, 0.001))

        with self.assertRaises(ValueError):
            self.controller(
                velocity_deadband_enter=0.04,
                velocity_deadband=0.03,
            )
        with self.assertRaises(ValueError):
            self.controller(velocity_deadband_enter=-0.01)

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

    def test_reset_clears_filter_history(self):
        controller = self.controller(
            True,
            position_lpf_tau_x=0.10,
            position_lpf_tau_y=0.10,
            position_lpf_tau_z=0.10,
        )
        controller.update((0.5, 0.0, 0.0), 0.3, 0.05)
        controller.reset()
        velocity, _, filtered = controller.update((-0.5, 0.0, 0.0), -0.3, 0.05)
        self.assertTrue(math.isclose(velocity[0], -0.2, abs_tol=1e-9))
        self.assertTrue(math.isclose(filtered, -0.3, abs_tol=1e-9))

    def test_position_low_pass_filters_each_axis_with_own_tau(self):
        controller = self.controller(
            kp_x=1.0,
            kp_y=1.0,
            kp_z=1.0,
            vx_limit=4.0,
            vyz_limit=4.0,
            velocity_deadband_enter=0.0,
            velocity_deadband=0.0,
            z_error_deadband=0.0,
            position_lpf_tau_x=0.0,
            position_lpf_tau_y=0.10,
            position_lpf_tau_z=0.30,
        )
        first, _, _ = controller.update((1.0, 1.0, 1.0), 0.0, 0.10)
        second, _, _ = controller.update((0.0, 0.0, 0.0), 0.0, 0.10)

        self.assertEqual(first, (1.0, 1.0, 1.0))
        self.assertTrue(math.isclose(second[0], 0.0, abs_tol=1e-12))
        self.assertTrue(math.isclose(second[1], 0.5, abs_tol=1e-12))
        self.assertTrue(math.isclose(second[2], 0.75, abs_tol=1e-12))

    def test_position_low_pass_precedes_gain_deadband_and_limits(self):
        controller = self.controller(
            kp_x=2.0,
            kp_y=1.0,
            kp_z=1.0,
            vx_limit=0.6,
            vyz_limit=0.5,
            velocity_deadband_enter=0.0,
            velocity_deadband=0.0,
            z_error_deadband=0.2,
            position_lpf_tau_x=0.10,
            position_lpf_tau_y=0.10,
            position_lpf_tau_z=0.10,
        )
        controller.update((0.0, 0.0, 0.0), 0.0, 0.10)
        velocity, _, _ = controller.update((1.0, 0.6, 0.3), 0.0, 0.10)

        self.assertTrue(math.isclose(velocity[0], 0.6, abs_tol=1e-12))
        self.assertTrue(math.isclose(velocity[1], 0.3, abs_tol=1e-12))
        self.assertEqual(velocity[2], 0.0)

    def test_position_low_pass_rejects_invalid_parameters(self):
        for name in (
            'position_lpf_tau_x',
            'position_lpf_tau_y',
            'position_lpf_tau_z',
        ):
            for invalid in (-0.01, float('nan'), float('inf')):
                with self.subTest(name=name, invalid=invalid):
                    with self.assertRaises(ValueError):
                        self.controller(**{name: invalid})

    def test_rejects_non_finite_errors_and_invalid_dt(self):
        controller = self.controller(True)
        with self.assertRaises(ValueError):
            controller.update((float('nan'), 0.0, 0.0), 0.0, 0.1)
        with self.assertRaises(ValueError):
            controller.update((0.1, 0.0, 0.0), 0.0, 0.0)
        with self.assertRaises(ValueError):
            controller.update((0.1, 0.0), 0.0, 0.1)


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
