#!/usr/bin/env python3

import math
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
import rospy
import rostest
from std_msgs.msg import String


def camera_pose(body_error, stamp, yaw_error):
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = 'usb_camera_optical_frame'
    message.pose.position.x = -float(body_error[1])
    message.pose.position.y = -float(body_error[2])
    message.pose.position.z = 0.8 + float(body_error[0])
    message.pose.orientation.x = math.cos(yaw_error * 0.5)
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = math.sin(yaw_error * 0.5)
    message.pose.orientation.w = 0.0
    return message


class YawControlSwitchTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.off_command = None
        self.on_command = None
        self.off_preview = None
        self.on_preview = None
        self.off_status = ''
        self.on_status = ''
        self.off_pose_pub = rospy.Publisher(
            '/test_yaw_control/off/pose', PoseStamped, queue_size=10
        )
        self.on_pose_pub = rospy.Publisher(
            '/test_yaw_control/on/pose', PoseStamped, queue_size=10
        )
        self.subscribers = [
            rospy.Subscriber(
                '/test_yaw_control/off/cmd_vel', TwistStamped,
                self._set_message, callback_args='off_command', queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/on/cmd_vel', TwistStamped,
                self._set_message, callback_args='on_command', queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/off/preview', PositionTarget,
                self._set_message, callback_args='off_preview', queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/on/preview', PositionTarget,
                self._set_message, callback_args='on_preview', queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/off/status', String,
                self._set_status, callback_args='off_status', queue_size=10,
            ),
            rospy.Subscriber(
                '/test_yaw_control/on/status', String,
                self._set_status, callback_args='on_status', queue_size=10,
            ),
        ]
        self._wait_for(
            lambda: self.off_pose_pub.get_num_connections() > 0
            and self.on_pose_pub.get_num_connections() > 0,
            5.0,
        )

    def _set_message(self, message, name):
        with self.lock:
            setattr(self, name, message)

    def _set_status(self, message, name):
        with self.lock:
            setattr(self, name, message.data)

    def _wait_for(self, predicate, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail('condition not met within %.1fs' % timeout)

    def _publish_stage(self, body_error, yaw_error, duration):
        deadline = time.time() + duration
        while time.time() < deadline and not rospy.is_shutdown():
            stamp = rospy.Time.now()
            message = camera_pose(body_error, stamp, yaw_error)
            self.off_pose_pub.publish(message)
            self.on_pose_pub.publish(message)
            rospy.sleep(0.05)

    def test_yaw_switch_and_alignment(self):
        self._publish_stage((0.3, 0.1, 0.1), 0.3, 0.5)
        self._wait_for(
            lambda: self.off_command is not None
            and self.on_command is not None
            and self.off_preview is not None
            and self.on_preview is not None,
            2.0,
        )
        with self.lock:
            off_command = self.off_command
            on_command = self.on_command
            off_preview = self.off_preview
            on_preview = self.on_preview
        self.assertGreater(abs(off_command.twist.linear.x), 0.0)
        self.assertGreater(abs(off_command.twist.linear.y), 0.0)
        self.assertGreater(abs(off_command.twist.linear.z), 0.0)
        self.assertEqual(off_command.twist.angular.z, 0.0)
        self.assertEqual(off_preview.yaw_rate, 0.0)
        self.assertTrue(math.isclose(on_command.twist.angular.z, 0.12, abs_tol=0.01))
        self.assertTrue(math.isclose(on_preview.yaw_rate, 0.12, abs_tol=0.01))

        self._publish_stage((0.0, 0.0, 0.0), 0.3, 2.5)
        self._wait_for(
            lambda: 'ALIGNED' in self.off_status
            and 'yaw_control=OFF' in self.off_status
            and 'yaw_control=ON' in self.on_status,
            3.0,
        )
        with self.lock:
            off_status = self.off_status
            on_status = self.on_status
        self.assertIn('ALIGNED', off_status)
        self.assertIn('yaw_control=OFF', off_status)
        self.assertIn('ALIGN', on_status)
        self.assertNotIn('ALIGNED', on_status)
        self.assertIn('yaw_control=ON', on_status)

        published_topics = dict(rospy.get_published_topics())
        self.assertNotIn('/test_yaw_control/off/actual', published_topics)
        self.assertNotIn('/test_yaw_control/on/actual', published_topics)


if __name__ == '__main__':
    rospy.init_node('yaw_control_switch_test')
    rostest.rosrun(
        'front_marker_control',
        'yaw_control_switch',
        YawControlSwitchTest,
    )
