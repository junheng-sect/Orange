#!/usr/bin/env python3
"""Verify fixed-marker filtered 6DoF and controller-error/yaw localization."""

import copy
import math
import threading
import unittest

from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
import rostest
from tf.transformations import (
    euler_from_quaternion,
    euler_matrix,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    quaternion_slerp,
    translation_matrix,
)
from tf2_msgs.msg import TFMessage


MAP_MARKER_TRANSLATION = (1.0, 0.0, 0.1)
MAP_MARKER_QUATERNION = (0.5, -0.5, -0.5, 0.5)
BASE_CAMERA_QUATERNION = (-0.5, 0.5, -0.5, 0.5)
FILTER_TAU_SEC = 0.25


def transform_matrix(translation, quaternion):
    matrix = quaternion_matrix(quaternion)
    matrix[:3, 3] = translation
    return matrix


def pose_from_matrix(matrix, stamp, frame_id):
    quaternion = quaternion_from_matrix(matrix)
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.pose.position.x = float(matrix[0, 3])
    message.pose.position.y = float(matrix[1, 3])
    message.pose.position.z = float(matrix[2, 3])
    message.pose.orientation.x = float(quaternion[0])
    message.pose.orientation.y = float(quaternion[1])
    message.pose.orientation.z = float(quaternion[2])
    message.pose.orientation.w = float(quaternion[3])
    return message


def camera_marker_from_map_base(map_to_base, base_to_camera):
    map_to_marker = transform_matrix(
        MAP_MARKER_TRANSLATION, MAP_MARKER_QUATERNION
    )
    return np.matmul(
        np.matmul(inverse_matrix(base_to_camera), inverse_matrix(map_to_base)),
        map_to_marker,
    )


class FixedMarkerBaseTfTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.transforms = {'test_fixed_base': [], 'test_error_base': []}
        self.missing_transforms = []
        self.pose_pub = rospy.Publisher(
            '/test_fixed_marker/pose', PoseStamped, queue_size=1
        )
        self.error_pose_pub = rospy.Publisher(
            '/test_fixed_marker/error_pose', PoseStamped, queue_size=1
        )
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback, queue_size=100)

    def tf_callback(self, message):
        with self.lock:
            for transform in message.transforms:
                if transform.child_frame_id in self.transforms:
                    self.transforms[transform.child_frame_id].append(transform)
                elif transform.child_frame_id == 'test_missing_base':
                    self.missing_transforms.append(transform)

    def wait_for_nodes(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if (
                self.pose_pub.get_num_connections() >= 2
                and self.error_pose_pub.get_num_connections() >= 1
            ):
                return
            rospy.sleep(0.05)
        self.fail('fixed-marker localization nodes did not subscribe')

    def assert_no_main_output(self):
        rospy.sleep(0.25)
        with self.lock:
            self.assertFalse(self.transforms['test_fixed_base'])
            self.assertFalse(self.missing_transforms)

    def wait_for_stamp(self, child_frame, stamp):
        deadline = rospy.Time.now() + rospy.Duration(4.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                matches = [
                    transform
                    for transform in self.transforms[child_frame]
                    if transform.header.stamp == stamp
                ]
            if matches:
                return matches[-1]
            rospy.sleep(0.02)
        self.fail('{} did not publish TF at requested stamp'.format(child_frame))

    @staticmethod
    def transform_components(transform):
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=float,
        )
        quaternion = np.asarray(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ],
            dtype=float,
        )
        return translation, quaternion

    def assert_transform_matches(self, transform, expected_matrix, places=6):
        actual_translation, actual_quaternion = self.transform_components(transform)
        expected_translation = np.asarray(expected_matrix[:3, 3], dtype=float)
        expected_quaternion = np.asarray(
            quaternion_from_matrix(expected_matrix), dtype=float
        )
        for actual, expected in zip(actual_translation, expected_translation):
            self.assertAlmostEqual(actual, expected, places=places)
        self.assertAlmostEqual(
            abs(float(np.dot(actual_quaternion, expected_quaternion))),
            1.0,
            places=places,
        )

    def test_error_yaw_position_and_heading(self):
        self.wait_for_nodes()
        base_to_camera = transform_matrix(
            (0.0, 0.0, 0.0), BASE_CAMERA_QUATERNION
        )
        expected_map_to_base = np.matmul(
            translation_matrix((0.30, 0.05, 0.12)),
            euler_matrix(0.0, 0.0, 0.35),
        )
        camera_to_marker = camera_marker_from_map_base(
            expected_map_to_base, base_to_camera
        )
        stamp = rospy.Time.now()
        self.error_pose_pub.publish(
            pose_from_matrix(camera_to_marker, stamp, 'test_error_camera')
        )
        received = self.wait_for_stamp('test_error_base', stamp)
        self.assertEqual(received.header.frame_id, 'test_fixed_map')
        self.assert_transform_matches(received, expected_map_to_base)

        _, quaternion = self.transform_components(received)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.35, places=6)

    def test_filtered_6d_validation_filter_gate_reset_and_no_hold(self):
        self.wait_for_nodes()

        wrong_frame = PoseStamped()
        wrong_frame.header.stamp = rospy.Time.now()
        wrong_frame.header.frame_id = 'wrong_camera'
        wrong_frame.pose.orientation.w = 1.0
        self.pose_pub.publish(wrong_frame)
        self.assert_no_main_output()

        zero_stamp = PoseStamped()
        zero_stamp.header.frame_id = 'test_fixed_camera'
        zero_stamp.pose.orientation.w = 1.0
        self.pose_pub.publish(zero_stamp)
        self.assert_no_main_output()

        non_finite = PoseStamped()
        non_finite.header.stamp = rospy.Time.now()
        non_finite.header.frame_id = 'test_fixed_camera'
        non_finite.pose.position.x = float('nan')
        non_finite.pose.orientation.w = 1.0
        self.pose_pub.publish(non_finite)
        self.assert_no_main_output()

        non_unit = PoseStamped()
        non_unit.header.stamp = rospy.Time.now()
        non_unit.header.frame_id = 'test_fixed_camera'
        non_unit.pose.orientation.w = 2.0
        self.pose_pub.publish(non_unit)
        self.assert_no_main_output()

        base_to_camera = transform_matrix(
            (0.0, 0.0, 0.0), BASE_CAMERA_QUATERNION
        )
        initial_map_to_base = np.matmul(
            translation_matrix((0.20, -0.10, 0.15)),
            euler_matrix(0.10, -0.05, 0.30),
        )
        initial_camera_to_marker = camera_marker_from_map_base(
            initial_map_to_base, base_to_camera
        )
        initial_stamp = rospy.Time.now()
        initial = pose_from_matrix(
            initial_camera_to_marker, initial_stamp, 'test_fixed_camera'
        )
        self.pose_pub.publish(initial)
        received = self.wait_for_stamp('test_fixed_base', initial_stamp)
        self.assertEqual(received.header.frame_id, 'test_fixed_map')
        self.assert_transform_matches(received, initial_map_to_base)

        # Negating a quaternion represents the same rotation and must not jump.
        sign_stamp = initial_stamp + rospy.Duration(0.05)
        sign_flipped = copy.deepcopy(initial)
        sign_flipped.header.stamp = sign_stamp
        sign_flipped.pose.orientation.x *= -1.0
        sign_flipped.pose.orientation.y *= -1.0
        sign_flipped.pose.orientation.z *= -1.0
        sign_flipped.pose.orientation.w *= -1.0
        self.pose_pub.publish(sign_flipped)
        sign_received = self.wait_for_stamp('test_fixed_base', sign_stamp)
        self.assert_transform_matches(sign_received, initial_map_to_base)

        step_map_to_base = np.matmul(
            translation_matrix((0.23, -0.10, 0.15)),
            euler_matrix(0.10, -0.05, 0.35),
        )
        step_camera_to_marker = camera_marker_from_map_base(
            step_map_to_base, base_to_camera
        )
        step_stamp = sign_stamp + rospy.Duration(0.10)
        self.pose_pub.publish(
            pose_from_matrix(step_camera_to_marker, step_stamp, 'test_fixed_camera')
        )
        step_received = self.wait_for_stamp('test_fixed_base', step_stamp)

        alpha = 1.0 - math.exp(-0.10 / FILTER_TAU_SEC)
        expected_translation = initial_map_to_base[:3, 3] + alpha * (
            step_map_to_base[:3, 3] - initial_map_to_base[:3, 3]
        )
        initial_quaternion = quaternion_from_matrix(initial_map_to_base)
        step_quaternion = quaternion_from_matrix(step_map_to_base)
        if float(np.dot(initial_quaternion, step_quaternion)) < 0.0:
            step_quaternion *= -1.0
        expected_quaternion = quaternion_slerp(
            initial_quaternion, step_quaternion, alpha, shortestpath=True
        )
        actual_translation, actual_quaternion = self.transform_components(step_received)
        for actual, expected in zip(actual_translation, expected_translation):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertAlmostEqual(
            abs(float(np.dot(actual_quaternion, expected_quaternion))),
            1.0,
            places=6,
        )

        with self.lock:
            count_before_outlier = len(self.transforms['test_fixed_base'])
        outlier_map_to_base = np.matmul(
            translation_matrix((0.50, -0.10, 0.15)),
            euler_matrix(0.10, -0.05, 0.35),
        )
        outlier_stamp = step_stamp + rospy.Duration(0.05)
        self.pose_pub.publish(
            pose_from_matrix(
                camera_marker_from_map_base(outlier_map_to_base, base_to_camera),
                outlier_stamp,
                'test_fixed_camera',
            )
        )
        rospy.sleep(0.25)
        with self.lock:
            self.assertEqual(
                len(self.transforms['test_fixed_base']), count_before_outlier
            )

        # A gap beyond reset_gap accepts the first new pose without old-state lag.
        reset_map_to_base = np.matmul(
            translation_matrix((0.50, 0.05, 0.20)),
            euler_matrix(-0.05, 0.04, 0.70),
        )
        reset_stamp = step_stamp + rospy.Duration(0.70)
        reset_message = pose_from_matrix(
            camera_marker_from_map_base(reset_map_to_base, base_to_camera),
            reset_stamp,
            'test_fixed_camera',
        )
        self.pose_pub.publish(reset_message)
        reset_received = self.wait_for_stamp('test_fixed_base', reset_stamp)
        self.assert_transform_matches(reset_received, reset_map_to_base)

        with self.lock:
            published_count = len(self.transforms['test_fixed_base'])
        self.pose_pub.publish(copy.deepcopy(reset_message))
        older = copy.deepcopy(reset_message)
        older.header.stamp = reset_stamp - rospy.Duration(0.01)
        self.pose_pub.publish(older)
        rospy.sleep(0.4)
        with self.lock:
            self.assertEqual(len(self.transforms['test_fixed_base']), published_count)
            self.assertFalse(self.missing_transforms)

        # Marker loss is silence: no timer republishes or holds map-to-base TF.
        rospy.sleep(0.6)
        with self.lock:
            self.assertEqual(len(self.transforms['test_fixed_base']), published_count)
            self.assertFalse(self.missing_transforms)


if __name__ == '__main__':
    rospy.init_node('test_fixed_marker_base_tf')
    rostest.rosrun(
        'seeker_visualization', 'fixed_marker_base_tf', FixedMarkerBaseTfTest
    )
