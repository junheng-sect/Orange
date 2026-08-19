import math
import unittest

import numpy as np

from front_marker_tracking.control import (
    BodyVelocityController,
    FirstOrderAngleLowPass,
    PIDController,
    limit_vector2,
    limit_vector3,
    wrap_angle,
)
from front_marker_tracking.transforms import (
    CALIBRATED_CAM1_BODY_R_CAMERA_OPTICAL,
    CALIBRATED_CAM1_BODY_T_CAMERA_OPTICAL,
    DEFAULT_BODY_R_CAMERA_OPTICAL,
    DEFAULT_BODY_T_CAMERA_OPTICAL,
    body_direction_to_camera,
    body_target_to_camera,
    camera_marker_to_body,
    camera_velocity_to_body,
    row_major_matrix,
    tracking_errors,
)


FRONT_FACING_ROTATION = np.diag([1.0, -1.0, -1.0])
CAM0_BODY_TRANSLATION = np.array(
    [0.037648389238371834, 0.03544184747017066, -0.0036823910191135277]
)


def camera_transform(translation, rotation=FRONT_FACING_ROTATION):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


class MathAndControlTest(unittest.TestCase):
    def test_default_usb_rotation_is_rigid_and_has_expected_axis_signs(self):
        rotation = DEFAULT_BODY_R_CAMERA_OPTICAL
        np.testing.assert_allclose(rotation.T.dot(rotation), np.eye(3), atol=1e-12)
        self.assertTrue(math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-12))
        np.testing.assert_allclose(DEFAULT_BODY_T_CAMERA_OPTICAL, np.zeros(3))
        right = camera_velocity_to_body([1.0, 0.0, 0.0], rotation)
        down = camera_velocity_to_body([0.0, 1.0, 0.0], rotation)
        forward = camera_velocity_to_body([0.0, 0.0, 1.0], rotation)
        np.testing.assert_allclose(right, [0.0, -1.0, 0.0])
        np.testing.assert_allclose(down, [0.0, 0.0, -1.0])
        np.testing.assert_allclose(forward, [1.0, 0.0, 0.0])

    def test_centered_front_facing_marker_at_0_8_metre_has_zero_error(self):
        position_error, yaw_error, _ = tracking_errors(
            camera_transform([0.0, 0.0, 0.8]),
            [0.0, 0.0, 0.8],
        )
        np.testing.assert_allclose(position_error, np.zeros(3))
        self.assertLess(abs(yaw_error), 1e-12)

    def test_0_8_metre_flu_target_maps_to_usb_optical_axis(self):
        target_body = np.array([0.8, 0.0, 0.0])
        target_camera = body_target_to_camera(
            target_body,
            DEFAULT_BODY_R_CAMERA_OPTICAL,
            DEFAULT_BODY_T_CAMERA_OPTICAL,
        )
        np.testing.assert_allclose(target_camera, [0.0, 0.0, 0.8], atol=1e-12)
        recovered_body = (
            DEFAULT_BODY_R_CAMERA_OPTICAL.dot(target_camera)
            + DEFAULT_BODY_T_CAMERA_OPTICAL
        )
        np.testing.assert_allclose(recovered_body, target_body, atol=1e-12)

    def test_cam0_and_cam1_have_distinct_physical_origins(self):
        baseline = CALIBRATED_CAM1_BODY_T_CAMERA_OPTICAL - CAM0_BODY_TRANSLATION
        self.assertGreater(float(np.linalg.norm(baseline)), 0.06)
        self.assertLess(float(np.linalg.norm(baseline)), 0.07)
        self.assertGreater(CAM0_BODY_TRANSLATION[1], 0.0)
        self.assertLess(CALIBRATED_CAM1_BODY_T_CAMERA_OPTICAL[1], 0.0)

    def test_left_and_right_observations_recover_same_body_marker_pose(self):
        angle = 0.23
        cosine = math.cos(angle)
        sine = math.sin(angle)
        body_t_marker = np.eye(4)
        body_t_marker[:3, :3] = np.array(
            [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
        ).dot(FRONT_FACING_ROTATION)
        body_t_marker[:3, 3] = [1.35, -0.12, 0.08]

        for body_t_camera in (
            CAM0_BODY_TRANSLATION,
            CALIBRATED_CAM1_BODY_T_CAMERA_OPTICAL,
        ):
            body_t_camera_matrix = np.eye(4)
            body_t_camera_matrix[:3, :3] = (
                CALIBRATED_CAM1_BODY_R_CAMERA_OPTICAL
            )
            body_t_camera_matrix[:3, 3] = body_t_camera
            camera_t_marker = np.linalg.inv(body_t_camera_matrix).dot(body_t_marker)
            recovered = camera_marker_to_body(
                camera_t_marker,
                CALIBRATED_CAM1_BODY_R_CAMERA_OPTICAL,
                body_t_camera,
            )
            np.testing.assert_allclose(recovered, body_t_marker, atol=1e-12)

    def test_camera_to_body_rejects_nonfinite_matrix(self):
        invalid = camera_transform([0.0, 0.0, 1.0])
        invalid[1, 3] = float('nan')
        with self.assertRaises(ValueError):
            camera_marker_to_body(
                invalid,
                CALIBRATED_CAM1_BODY_R_CAMERA_OPTICAL,
                CAM0_BODY_TRANSLATION,
            )

    def test_body_aligned_marker_has_zero_position_and_yaw_error(self):
        target_camera = body_target_to_camera(
            [0.8, 0.0, 0.0],
            DEFAULT_BODY_R_CAMERA_OPTICAL,
            DEFAULT_BODY_T_CAMERA_OPTICAL,
        )
        body_forward_camera = body_direction_to_camera(
            [1.0, 0.0, 0.0], DEFAULT_BODY_R_CAMERA_OPTICAL
        )
        target_yaw = math.atan2(
            float(body_forward_camera[0]), float(body_forward_camera[2])
        )
        cosine = math.cos(target_yaw)
        sine = math.sin(target_yaw)
        rotation_y = np.array(
            [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]
        )
        transform = camera_transform(
            target_camera, rotation_y.dot(FRONT_FACING_ROTATION)
        )
        position_error, yaw_error, _ = tracking_errors(
            transform, target_camera, body_forward_camera
        )
        np.testing.assert_allclose(position_error, np.zeros(3), atol=1e-12)
        self.assertLess(abs(yaw_error), 1e-12)

    def test_legacy_calibrated_cam1_requires_body_yaw_compensation(self):
        target_camera = body_target_to_camera(
            [1.0, 0.0, 0.0],
            CALIBRATED_CAM1_BODY_R_CAMERA_OPTICAL,
            CALIBRATED_CAM1_BODY_T_CAMERA_OPTICAL,
        )
        body_forward_camera = body_direction_to_camera(
            [1.0, 0.0, 0.0], CALIBRATED_CAM1_BODY_R_CAMERA_OPTICAL
        )
        _, yaw_error, _ = tracking_errors(
            camera_transform(target_camera), target_camera, body_forward_camera
        )
        self.assertTrue(
            math.isclose(math.degrees(yaw_error), -0.7991838804, abs_tol=1e-9)
        )

    def test_invalid_body_translation_is_rejected(self):
        with self.assertRaises(ValueError):
            body_target_to_camera(
                [1.0, 0.0, 0.0],
                DEFAULT_BODY_R_CAMERA_OPTICAL,
                [float('nan'), 0.0, 0.0],
            )

    def test_right_below_far_marker_generates_expected_flu_signs(self):
        error, _, _ = tracking_errors(camera_transform([0.2, 0.1, 1.5]))
        body = camera_velocity_to_body(error, DEFAULT_BODY_R_CAMERA_OPTICAL)
        self.assertGreater(body[0], 0.0)
        self.assertLess(body[1], 0.0)
        self.assertLess(body[2], 0.0)

    def test_left_above_near_marker_generates_opposite_flu_signs(self):
        error, _, _ = tracking_errors(camera_transform([-0.2, -0.1, 0.5]))
        body = camera_velocity_to_body(error, DEFAULT_BODY_R_CAMERA_OPTICAL)
        self.assertLess(body[0], 0.0)
        self.assertGreater(body[1], 0.0)
        self.assertGreater(body[2], 0.0)

    def test_yaw_error_uses_marker_normal(self):
        for angle in (0.35, -0.35):
            cosine = math.cos(angle)
            sine = math.sin(angle)
            rotation_y = np.array(
                [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]
            )
            transform = camera_transform([0.0, 0.0, 1.0], rotation_y.dot(FRONT_FACING_ROTATION))
            _, yaw_error, _ = tracking_errors(transform)
            self.assertTrue(math.isclose(yaw_error, -angle, abs_tol=1e-12))

    def test_invalid_transform_is_rejected(self):
        invalid = camera_transform([0.0, 0.0, 1.0])
        invalid[0, 0] = 2.0
        with self.assertRaises(ValueError):
            row_major_matrix(invalid.reshape(16))

    def test_pid_and_vector_limit(self):
        pid = PIDController(0.3, 0.0, 0.0)
        self.assertTrue(math.isclose(pid.update(0.5, 0.1), 0.15))
        vector = limit_vector3(3.0, 4.0, 0.0, 0.3)
        self.assertTrue(
            math.isclose(math.sqrt(sum(value * value for value in vector)), 0.3)
        )

    def test_pid_derivative_uses_explicit_observation_interval(self):
        pid = PIDController(0.0, 0.0, 1.0)
        self.assertTrue(math.isclose(pid.update(1.0, 0.05), 0.0))
        self.assertTrue(math.isclose(pid.update(1.5, 0.05), 10.0))
        pid.reset()
        self.assertTrue(math.isclose(pid.update(1.5, 0.05), 0.0))

    def test_angle_low_pass_20hz_response_and_reset(self):
        low_pass = FirstOrderAngleLowPass(0.10)
        self.assertTrue(math.isclose(low_pass.update(0.0, 0.05), 0.0))
        self.assertTrue(math.isclose(low_pass.update(1.0, 0.05), 1.0 / 3.0))
        self.assertTrue(math.isclose(low_pass.update(1.0, 0.05), 5.0 / 9.0))
        low_pass.reset()
        self.assertTrue(math.isclose(low_pass.update(-0.7, 0.05), -0.7))

    def test_angle_low_pass_tau_zero_bypasses_filter(self):
        low_pass = FirstOrderAngleLowPass(0.0)
        self.assertTrue(math.isclose(low_pass.update(0.8, 0.05), 0.8))
        self.assertTrue(math.isclose(low_pass.update(-0.4, 0.05), -0.4))

    def test_angle_low_pass_uses_shortest_delta_across_pi(self):
        low_pass = FirstOrderAngleLowPass(0.10)
        first = low_pass.update(math.radians(179.0), 0.05)
        second = low_pass.update(math.radians(-179.0), 0.05)
        self.assertGreater(first, 0.0)
        self.assertGreater(abs(second), math.radians(170.0))
        self.assertLess(
            abs(wrap_angle(second - first)),
            math.radians(1.0),
        )

    def test_angle_low_pass_rejects_invalid_tau_and_dt(self):
        with self.assertRaises(ValueError):
            FirstOrderAngleLowPass(-0.01)
        with self.assertRaises(ValueError):
            FirstOrderAngleLowPass(float('nan'))
        low_pass = FirstOrderAngleLowPass(0.10)
        with self.assertRaises(ValueError):
            low_pass.update(0.1, 0.0)

    def test_yz_vector_limit_preserves_direction(self):
        limited_y, limited_z = limit_vector2(0.6, 0.8, 0.5)
        self.assertTrue(math.isclose(math.hypot(limited_y, limited_z), 0.5))
        self.assertTrue(math.isclose(limited_y / limited_z, 0.75))

    def make_body_controller(self, enable_yaw_control=True):
        return BodyVelocityController(
            kp_x=0.5,
            ki_x=0.0,
            kd_x=0.03,
            kp_yz=0.6,
            ki_yz=0.0,
            kd_yz=0.08,
            kp_yaw=0.8,
            ki_yaw=0.0,
            kd_yaw=0.0,
            vx_limit=0.2,
            v_limit=0.8,
            yaw_rate_limit=0.8,
            velocity_deadband=0.03,
            z_error_deadband=0.04,
            yaw_error_deadband=0.04,
            yaw_lpf_tau=0.10,
            enable_yaw_control=enable_yaw_control,
        )

    def test_body_controller_limits_x_yz_and_yaw(self):
        controller = self.make_body_controller()
        body_velocity, yaw_rate, filtered_yaw_error = controller.update(
            [1.0, 1.0, 1.0], 2.0, 0.05
        )
        self.assertTrue(math.isclose(body_velocity[0], 0.2))
        self.assertTrue(
            math.isclose(math.hypot(body_velocity[1], body_velocity[2]), 0.8)
        )
        self.assertTrue(math.isclose(yaw_rate, 0.8))
        self.assertTrue(math.isclose(filtered_yaw_error, 2.0))

    def test_body_controller_deadbands_and_filter_state(self):
        controller = self.make_body_controller()
        _, first_yaw_rate, first_filtered_yaw = controller.update(
            [0.0, 0.0, 0.2], 0.2, 0.05
        )
        self.assertTrue(math.isclose(first_yaw_rate, 0.16))
        self.assertTrue(math.isclose(first_filtered_yaw, 0.2))

        body_velocity, yaw_rate, filtered_yaw_error = controller.update(
            [0.0, 0.0, 0.039], 0.039, 0.05
        )
        self.assertEqual(body_velocity, (0.0, 0.0, 0.0))
        self.assertTrue(math.isclose(filtered_yaw_error, 0.14633333333333334))
        self.assertTrue(math.isclose(yaw_rate, 0.11706666666666668))

        for _ in range(10):
            _, yaw_rate, filtered_yaw_error = controller.update(
                [0.0, 0.0, 0.0], 0.0, 0.05
            )
        self.assertLess(abs(filtered_yaw_error), 0.04)
        self.assertEqual(yaw_rate, 0.0)

        controller.reset()
        body_velocity, yaw_rate, filtered_yaw_error = controller.update(
            [0.0, 0.0, 0.2], 0.2, 0.05
        )
        self.assertTrue(math.isclose(body_velocity[2], 0.12))
        self.assertTrue(math.isclose(yaw_rate, 0.16))
        self.assertTrue(math.isclose(filtered_yaw_error, 0.2))

    def test_body_controller_output_deadband_applies_after_limits(self):
        controller = self.make_body_controller()
        body_velocity, yaw_rate, filtered_yaw_error = controller.update(
            [0.05, 0.04, 0.041], 0.041, 0.05
        )
        self.assertEqual(body_velocity, (0.0, 0.0, 0.0))
        self.assertTrue(math.isclose(yaw_rate, 0.0328))
        self.assertTrue(math.isclose(filtered_yaw_error, 0.041))

    def test_disabled_yaw_control_preserves_observation_without_pid_state(self):
        controller = self.make_body_controller(enable_yaw_control=False)
        body_velocity, yaw_rate, filtered_yaw_error = controller.update(
            [1.0, 0.2, 0.2], 0.3, 0.05
        )
        self.assertTrue(math.isclose(body_velocity[0], 0.2))
        self.assertGreater(abs(body_velocity[1]), 0.0)
        self.assertGreater(abs(body_velocity[2]), 0.0)
        self.assertEqual(yaw_rate, 0.0)
        self.assertTrue(math.isclose(filtered_yaw_error, 0.3))
        self.assertEqual(controller.pid_yaw.integral, 0.0)
        self.assertIsNone(controller.pid_yaw.previous_error)

    def test_position_target_mask_enables_velocity_and_yaw_rate_only(self):
        try:
            from mavros_msgs.msg import PositionTarget
            from front_marker_tracking.mavros_interface import velocity_only_type_mask
        except ImportError:
            self.skipTest('mavros_msgs is unavailable in this host environment')

        mask = velocity_only_type_mask()
        self.assertFalse(mask & PositionTarget.IGNORE_VX)
        self.assertFalse(mask & PositionTarget.IGNORE_VY)
        self.assertFalse(mask & PositionTarget.IGNORE_VZ)
        self.assertFalse(mask & PositionTarget.IGNORE_YAW_RATE)
        self.assertTrue(mask & PositionTarget.IGNORE_PX)
        self.assertTrue(mask & PositionTarget.IGNORE_YAW)


if __name__ == '__main__':
    unittest.main()
