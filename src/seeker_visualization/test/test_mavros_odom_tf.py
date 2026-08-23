#!/usr/bin/env python3
"""Verify validation and exact Odometry-to-TF forwarding."""

import math
import threading
import unittest

from nav_msgs.msg import Odometry
import rospy
import rostest
from tf2_msgs.msg import TFMessage


class MavrosOdometryTfTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.transforms = []
        self.odom_pub = rospy.Publisher(
            '/test_seeker_visualization/odom', Odometry, queue_size=1
        )
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback, queue_size=20)

    def tf_callback(self, message):
        with self.lock:
            self.transforms.extend(
                transform
                for transform in message.transforms
                if transform.child_frame_id == 'test_base_link'
            )

    @staticmethod
    def odometry(parent='test_map', child='test_base_link'):
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.pose.pose.position.x = 1.25
        message.pose.pose.position.y = -0.5
        message.pose.pose.position.z = 0.75
        message.pose.pose.orientation.z = math.sin(0.3)
        message.pose.pose.orientation.w = math.cos(0.3)
        return message

    def wait_for_node(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.odom_pub.get_num_connections() > 0:
                return
            rospy.sleep(0.05)
        self.fail('Odometry TF node did not subscribe')

    def test_validates_and_forwards_exact_pose(self):
        self.wait_for_node()

        self.odom_pub.publish(self.odometry(parent='wrong_map'))
        invalid_quaternion = self.odometry()
        invalid_quaternion.pose.pose.orientation.z = 0.0
        invalid_quaternion.pose.pose.orientation.w = 2.0
        self.odom_pub.publish(invalid_quaternion)
        rospy.sleep(0.3)
        with self.lock:
            self.assertFalse(self.transforms, 'invalid Odometry produced TF')

        expected = self.odometry()
        self.odom_pub.publish(expected)
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        received = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.transforms:
                    received = self.transforms[-1]
            if received is not None:
                break
            rospy.sleep(0.02)

        self.assertIsNotNone(received, 'valid Odometry did not produce TF')
        self.assertEqual(received.header.stamp, expected.header.stamp)
        self.assertEqual(received.header.frame_id, expected.header.frame_id)
        self.assertEqual(received.child_frame_id, expected.child_frame_id)
        self.assertEqual(
            received.transform.translation.x, expected.pose.pose.position.x
        )
        self.assertEqual(
            received.transform.translation.y, expected.pose.pose.position.y
        )
        self.assertEqual(
            received.transform.translation.z, expected.pose.pose.position.z
        )
        self.assertEqual(
            received.transform.rotation.z, expected.pose.pose.orientation.z
        )
        self.assertEqual(
            received.transform.rotation.w, expected.pose.pose.orientation.w
        )


if __name__ == '__main__':
    rospy.init_node('test_mavros_odom_tf')
    rostest.rosrun(
        'seeker_visualization',
        'mavros_odom_tf',
        MavrosOdometryTfTest,
    )
