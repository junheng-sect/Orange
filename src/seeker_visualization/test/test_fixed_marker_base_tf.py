#!/usr/bin/env python3
"""Verify fixed-marker 6DoF base inversion and rejection behavior."""

import copy
import math
import threading
import unittest

from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
import rostest
from tf.transformations import (
    euler_matrix,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)
from tf2_msgs.msg import TFMessage


MAP_MARKER_TRANSLATION = (1.0, 0.0, 0.1)
MAP_MARKER_QUATERNION = (0.5, -0.5, -0.5, 0.5)
BASE_CAMERA_QUATERNION = (-0.5, 0.5, -0.5, 0.5)


def transform_matrix(translation, quaternion):
    matrix = quaternion_matrix(quaternion)
    matrix[:3, 3] = translation
    return matrix


def pose_from_matrix(matrix, stamp, frame_id='test_fixed_camera'):
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


class FixedMarkerBaseTfTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.transforms = []
        self.missing_transforms = []
        self.pose_pub = rospy.Publisher(
            '/test_fixed_marker/pose', PoseStamped, queue_size=1
        )
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback, queue_size=50)

    def tf_callback(self, message):
        with self.lock:
            for transform in message.transforms:
                if transform.child_frame_id == 'test_fixed_base':
                    self.transforms.append(transform)
                elif transform.child_frame_id == 'test_missing_base':
                    self.missing_transforms.append(transform)

    def wait_for_nodes(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.pose_pub.get_num_connections() >= 2:
                return
            rospy.sleep(0.05)
        self.fail('fixed-marker localization nodes did not subscribe')

    def assert_no_output(self):
        rospy.sleep(0.25)
        with self.lock:
            self.assertFalse(self.transforms)
            self.assertFalse(self.missing_transforms)

    def wait_for_stamp(self, stamp):
        deadline = rospy.Time.now() + rospy.Duration(4.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                matches = [
                    transform
                    for transform in self.transforms
                    if transform.header.stamp == stamp
                ]
            if matches:
                return matches[-1]
            rospy.sleep(0.02)
        self.fail('valid marker Pose did not produce map-to-base TF')

    def test_validation_inversion_and_no_hold(self):
        self.wait_for_nodes()

        wrong_frame = PoseStamped()
        wrong_frame.header.stamp = rospy.Time.now()
        wrong_frame.header.frame_id = 'wrong_camera'
        wrong_frame.pose.orientation.w = 1.0
        self.pose_pub.publish(wrong_frame)
        self.assert_no_output()

        zero_stamp = PoseStamped()
        zero_stamp.header.frame_id = 'test_fixed_camera'
        zero_stamp.pose.orientation.w = 1.0
        self.pose_pub.publish(zero_stamp)
        self.assert_no_output()

        non_finite = PoseStamped()
        non_finite.header.stamp = rospy.Time.now()
        non_finite.header.frame_id = 'test_fixed_camera'
        non_finite.pose.position.x = float('nan')
        non_finite.pose.orientation.w = 1.0
        self.pose_pub.publish(non_finite)
        self.assert_no_output()

        non_unit = PoseStamped()
        non_unit.header.stamp = rospy.Time.now()
        non_unit.header.frame_id = 'test_fixed_camera'
        non_unit.pose.orientation.w = 2.0
        self.pose_pub.publish(non_unit)
        self.assert_no_output()

        map_to_marker = transform_matrix(
            MAP_MARKER_TRANSLATION, MAP_MARKER_QUATERNION
        )
        base_to_camera = transform_matrix((0.0, 0.0, 0.0), BASE_CAMERA_QUATERNION)
        expected_map_to_base = np.matmul(
            translation_matrix((0.2, -0.1, 0.15)),
            euler_matrix(0.1, -0.05, 0.3),
        )
        camera_to_marker = np.matmul(
            np.matmul(inverse_matrix(base_to_camera), inverse_matrix(expected_map_to_base)),
            map_to_marker,
        )
        stamp = rospy.Time.now()
        valid = pose_from_matrix(camera_to_marker, stamp)
        self.pose_pub.publish(valid)
        received = self.wait_for_stamp(stamp)

        self.assertEqual(received.header.frame_id, 'test_fixed_map')
        self.assertEqual(received.child_frame_id, 'test_fixed_base')
        actual_translation = (
            received.transform.translation.x,
            received.transform.translation.y,
            received.transform.translation.z,
        )
        for actual, expected in zip(actual_translation, expected_map_to_base[:3, 3]):
            self.assertAlmostEqual(actual, expected, places=6)

        actual_quaternion = np.array(
            [
                received.transform.rotation.x,
                received.transform.rotation.y,
                received.transform.rotation.z,
                received.transform.rotation.w,
            ]
        )
        expected_quaternion = quaternion_from_matrix(expected_map_to_base)
        self.assertAlmostEqual(
            abs(float(np.dot(actual_quaternion, expected_quaternion))), 1.0, places=6
        )

        with self.lock:
            published_count = len(self.transforms)
        self.pose_pub.publish(copy.deepcopy(valid))
        older = copy.deepcopy(valid)
        older.header.stamp = stamp - rospy.Duration(0.01)
        self.pose_pub.publish(older)
        rospy.sleep(0.4)
        with self.lock:
            self.assertEqual(len(self.transforms), published_count)
            self.assertFalse(self.missing_transforms)

        # Marker loss is represented by silence: no timer republishes or holds TF.
        rospy.sleep(0.6)
        with self.lock:
            self.assertEqual(len(self.transforms), published_count)
            self.assertFalse(self.missing_transforms)


if __name__ == '__main__':
    rospy.init_node('test_fixed_marker_base_tf')
    rostest.rosrun(
        'seeker_visualization', 'fixed_marker_base_tf', FixedMarkerBaseTfTest
    )
