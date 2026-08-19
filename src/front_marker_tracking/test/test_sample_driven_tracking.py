#!/usr/bin/env python3

import math
import threading
import time
import unittest

from front_marker_tracking.msg import HomogeneousTransform
from front_marker_tracking.transforms import (
    DEFAULT_BODY_R_CAMERA_OPTICAL,
    DEFAULT_BODY_T_CAMERA_OPTICAL,
    body_direction_to_camera,
    body_target_to_camera,
)
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget
import numpy as np
import rospy
import rostest


FRONT_FACING_ROTATION = np.diag([1.0, -1.0, -1.0])


class SampleDrivenTrackingTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.body_commands = []
        self.errors = []
        self.filtered_errors = []
        self.preview_receive_times = []
        self.transform_pub = rospy.Publisher(
            '/test_front_marker_pid/transform',
            HomogeneousTransform,
            queue_size=10,
        )
        self.body_sub = rospy.Subscriber(
            '/front_marker_tracking/cmd_vel_flu',
            TwistStamped,
            self.body_command_callback,
            queue_size=100,
        )
        self.error_sub = rospy.Subscriber(
            '/front_marker_tracking/error_flu',
            TwistStamped,
            self.error_callback,
            queue_size=100,
        )
        self.filtered_error_sub = rospy.Subscriber(
            '/front_marker_tracking/error_flu_filtered',
            TwistStamped,
            self.filtered_error_callback,
            queue_size=100,
        )
        self.preview_sub = rospy.Subscriber(
            '/test_front_marker_pid/setpoint_preview',
            PositionTarget,
            self.preview_callback,
            queue_size=100,
        )
        self.wait_for(
            lambda: self.transform_pub.get_num_connections() > 0
            and self.body_sub.get_num_connections() > 0
            and self.error_sub.get_num_connections() > 0
            and self.filtered_error_sub.get_num_connections() > 0
            and self.preview_sub.get_num_connections() > 0,
            5.0,
        )

    def body_command_callback(self, message):
        with self.lock:
            self.body_commands.append(message)

    def error_callback(self, message):
        with self.lock:
            self.errors.append(message)

    def filtered_error_callback(self, message):
        with self.lock:
            self.filtered_errors.append(message)

    def preview_callback(self, _message):
        with self.lock:
            self.preview_receive_times.append(rospy.get_time())

    def wait_for(self, predicate, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail('condition not met within %.1fs' % timeout)

    def make_transform(self, body_error, stamp, yaw_error=0.0):
        target_camera = body_target_to_camera(
            [0.8, 0.0, 0.0],
            DEFAULT_BODY_R_CAMERA_OPTICAL,
            DEFAULT_BODY_T_CAMERA_OPTICAL,
        )
        camera_error = DEFAULT_BODY_R_CAMERA_OPTICAL.T.dot(
            np.asarray(body_error, dtype=np.float64)
        )
        transform = np.eye(4)
        body_forward_camera = body_direction_to_camera(
            [1.0, 0.0, 0.0], DEFAULT_BODY_R_CAMERA_OPTICAL
        )
        target_yaw = math.atan2(
            float(body_forward_camera[0]),
            float(body_forward_camera[2]),
        )
        marker_yaw = target_yaw - float(yaw_error)
        cosine = math.cos(marker_yaw)
        sine = math.sin(marker_yaw)
        rotation_y = np.array(
            [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]
        )
        transform[:3, :3] = rotation_y.dot(FRONT_FACING_ROTATION)
        transform[:3, 3] = target_camera + camera_error

        message = HomogeneousTransform()
        message.header.stamp = stamp
        message.header.frame_id = 'usb_camera_optical_frame'
        message.marker_id = 33
        message.matrix = transform.reshape(16).tolist()
        return message

    def latest_body_x(self):
        with self.lock:
            if not self.body_commands:
                return None
            return self.body_commands[-1].twist.linear.x

    def latest_error_x(self):
        with self.lock:
            if not self.errors:
                return None
            return self.errors[-1].twist.linear.x

    def latest_filtered_yaw(self):
        with self.lock:
            if not self.filtered_errors:
                return None
            return self.filtered_errors[-1].twist.angular.z

    def test_sample_driven_pid_and_20hz_heartbeat(self):
        first_stamp = rospy.Time.now()
        self.transform_pub.publish(
            self.make_transform([0.5, 0.0, 0.0], first_stamp)
        )
        self.wait_for(
            lambda: self.latest_body_x() is not None
            and math.isclose(self.latest_body_x(), 0.2, abs_tol=1e-6),
            2.0,
        )
        self.wait_for(lambda: self.latest_error_x() is not None, 2.0)
        self.assertTrue(math.isclose(self.latest_error_x(), 0.5, abs_tol=1e-6))

        # A duplicate source stamp with different data must not update either
        # the error or PID state.
        self.transform_pub.publish(
            self.make_transform([-0.5, 0.0, 0.0], first_stamp)
        )
        rospy.sleep(0.15)
        self.assertTrue(math.isclose(self.latest_body_x(), 0.2, abs_tol=1e-6))
        self.assertTrue(math.isclose(self.latest_error_x(), 0.5, abs_tol=1e-6))

        second_stamp = rospy.Time.now()
        self.transform_pub.publish(
            self.make_transform([-0.5, 0.0, 0.0], second_stamp)
        )
        self.wait_for(
            lambda: math.isclose(self.latest_body_x(), -0.2, abs_tol=1e-6),
            2.0,
        )

        self.transform_pub.publish(
            self.make_transform([0.5, 0.0, 0.0], first_stamp)
        )
        self.wait_for(
            lambda: math.isclose(self.latest_body_x(), 0.0, abs_tol=1e-9),
            1.0,
        )
        self.transform_pub.publish(
            self.make_transform([-0.5, 0.0, 0.0], rospy.Time.now())
        )
        self.wait_for(
            lambda: math.isclose(self.latest_body_x(), -0.2, abs_tol=1e-6),
            2.0,
        )

        with self.lock:
            rate_start = len(self.preview_receive_times)
        end_time = time.time() + 0.45
        while time.time() < end_time:
            self.transform_pub.publish(
                self.make_transform([-0.5, 0.0, 0.0], rospy.Time.now())
            )
            rospy.sleep(0.05)
        with self.lock:
            times = self.preview_receive_times[rate_start:]
        self.assertGreaterEqual(len(times), 7)
        intervals = np.diff(times)
        rate = (len(times) - 1) / (times[-1] - times[0])
        self.assertGreaterEqual(rate, 19.0)
        self.assertLessEqual(rate, 21.0)
        self.assertLess(float(np.max(intervals)), 0.1)

        # No new observation for longer than pose_timeout_sec must reset the
        # held command to zero while the 20 Hz heartbeat continues.
        self.wait_for(
            lambda: math.isclose(self.latest_body_x(), 0.0, abs_tol=1e-9),
            1.5,
        )

        # Timeout reset clears the yaw filter. The first recovered observation
        # therefore passes through without stale history.
        yaw_stamp = rospy.Time.now()
        self.transform_pub.publish(
            self.make_transform([0.0, 0.0, 0.0], yaw_stamp, yaw_error=0.3)
        )
        self.wait_for(
            lambda: self.latest_filtered_yaw() is not None
            and math.isclose(self.latest_filtered_yaw(), 0.3, abs_tol=1e-6),
            2.0,
        )

        # A duplicate source stamp must not update the filter.
        self.transform_pub.publish(
            self.make_transform([0.0, 0.0, 0.0], yaw_stamp, yaw_error=-0.3)
        )
        rospy.sleep(0.15)
        self.assertTrue(
            math.isclose(self.latest_filtered_yaw(), 0.3, abs_tol=1e-6)
        )

        # A backward stamp invalidates and resets control. The next fresh
        # observation is another first frame and must pass through directly.
        self.transform_pub.publish(
            self.make_transform([0.0, 0.0, 0.0], first_stamp, yaw_error=0.1)
        )
        rospy.sleep(0.15)
        self.assertTrue(math.isclose(self.latest_body_x(), 0.0, abs_tol=1e-9))
        self.transform_pub.publish(
            self.make_transform(
                [0.0, 0.0, 0.0],
                rospy.Time.now(),
                yaw_error=-0.3,
            )
        )
        self.wait_for(
            lambda: math.isclose(self.latest_filtered_yaw(), -0.3, abs_tol=1e-6),
            2.0,
        )

        published_topics = dict(rospy.get_published_topics())
        self.assertNotIn('/mavros/setpoint_raw/local', published_topics)


if __name__ == '__main__':
    rospy.init_node('sample_driven_tracking_test')
    rostest.rosrun(
        'front_marker_tracking',
        'sample_driven_tracking',
        SampleDrivenTrackingTest,
    )
