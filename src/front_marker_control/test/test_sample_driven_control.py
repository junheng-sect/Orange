#!/usr/bin/env python3

import math
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
import rospy
import rostest


def camera_pose(body_error, stamp, yaw_error=0.0, frame='usb_camera_optical_frame'):
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame
    message.pose.position.x = -float(body_error[1])
    message.pose.position.y = -float(body_error[2])
    message.pose.position.z = 0.8 + float(body_error[0])
    message.pose.orientation.x = math.cos(yaw_error * 0.5)
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = math.sin(yaw_error * 0.5)
    message.pose.orientation.w = 0.0
    return message


class SampleDrivenControlTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.commands = []
        self.errors = []
        self.previews = []
        self.actual = []
        self.pose_pub = rospy.Publisher(
            '/test_front_marker_control/pose', PoseStamped, queue_size=10
        )
        self.state_pub = rospy.Publisher(
            '/test_front_marker_control/state', State, queue_size=10
        )
        rospy.Subscriber(
            '/test_front_marker_control/cmd_vel',
            TwistStamped,
            self._append,
            callback_args=self.commands,
            queue_size=100,
        )
        rospy.Subscriber(
            '/test_front_marker_control/error',
            Vector3Stamped,
            self._append,
            callback_args=self.errors,
            queue_size=100,
        )
        rospy.Subscriber(
            '/test_front_marker_control/preview',
            PositionTarget,
            self._append,
            callback_args=self.previews,
            queue_size=100,
        )
        rospy.Subscriber(
            '/test_front_marker_control/actual',
            PositionTarget,
            self._append,
            callback_args=self.actual,
            queue_size=100,
        )
        self.wait_for(
            lambda: self.pose_pub.get_num_connections() > 0
            and self.state_pub.get_num_connections() > 0,
            5.0,
        )

    def _append(self, message, target):
        with self.lock:
            target.append(message)

    def wait_for(self, predicate, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail('condition not met within %.1fs' % timeout)

    def latest_command_x(self):
        with self.lock:
            return None if not self.commands else self.commands[-1].twist.linear.x

    def latest_actual_x(self):
        with self.lock:
            return None if not self.actual else self.actual[-1].velocity.x

    def publish_state(self, connected, armed, mode):
        message = State()
        message.connected = connected
        message.armed = armed
        message.mode = mode
        self.state_pub.publish(message)

    def test_pose_validation_pid_heartbeat_and_mavros_gates(self):
        first_stamp = rospy.Time.now()
        self.pose_pub.publish(camera_pose((0.5, 0.0, 0.0), first_stamp))
        self.wait_for(
            lambda: self.latest_command_x() is not None
            and math.isclose(self.latest_command_x(), 0.2, abs_tol=1e-6),
            2.0,
        )
        self.wait_for(
            lambda: self.latest_actual_x() is not None
            and math.isclose(self.latest_actual_x(), 0.0, abs_tol=1e-9),
            2.0,
        )

        # Duplicate and backward stamps are rejected and reset the held command.
        self.pose_pub.publish(camera_pose((-0.5, 0.0, 0.0), first_stamp))
        self.wait_for(lambda: math.isclose(self.latest_command_x(), 0.0, abs_tol=1e-9), 1.0)
        self.pose_pub.publish(camera_pose((0.5, 0.0, 0.0), first_stamp - rospy.Duration(0.1)))
        self.wait_for(lambda: math.isclose(self.latest_command_x(), 0.0, abs_tol=1e-9), 1.0)

        # A frame without TF is rejected.
        self.pose_pub.publish(camera_pose((0.5, 0.0, 0.0), rospy.Time.now(), frame='missing_frame'))
        self.wait_for(lambda: math.isclose(self.latest_command_x(), 0.0, abs_tol=1e-9), 1.0)

        # Each MAVROS gate keeps actual output at zero until all are satisfied.
        stages = [
            (False, False, '', 0.0),
            (True, False, '', 0.0),
            (True, True, 'MANUAL', 0.0),
            (True, True, 'OFFBOARD', 0.2),
        ]
        for connected, armed, mode, expected in stages:
            self.publish_state(connected, armed, mode)
            rospy.sleep(0.05)
            self.pose_pub.publish(camera_pose((0.5, 0.0, 0.0), rospy.Time.now()))
            self.wait_for(
                lambda expected=expected: math.isclose(
                    self.latest_actual_x(), expected, abs_tol=1e-6
                ),
                2.0,
            )

        # Command and actual setpoint return to zero when Pose becomes stale.
        self.wait_for(lambda: math.isclose(self.latest_command_x(), 0.0, abs_tol=1e-9), 1.5)
        self.wait_for(lambda: math.isclose(self.latest_actual_x(), 0.0, abs_tol=1e-9), 1.5)

        with self.lock:
            receive_times = [item.header.stamp.to_sec() for item in self.previews[-10:]]
        self.assertGreaterEqual(len(receive_times), 7)
        rate = (len(receive_times) - 1) / (receive_times[-1] - receive_times[0])
        self.assertGreaterEqual(rate, 18.0)
        self.assertLessEqual(rate, 22.0)


if __name__ == '__main__':
    rospy.init_node('sample_driven_control_test')
    rostest.rosrun(
        'front_marker_control',
        'sample_driven_control',
        SampleDrivenControlTest,
    )
