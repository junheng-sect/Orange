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
from std_msgs.msg import String


FRONT_FACING_ROTATION = np.diag([1.0, -1.0, -1.0])


class YawControlSwitchTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.off_body = None
        self.on_body = None
        self.off_preview = None
        self.on_preview = None
        self.off_filtered = None
        self.off_status = ''
        self.on_status = ''

        self.off_transform_pub = rospy.Publisher(
            '/test_yaw_control/off/transform',
            HomogeneousTransform,
            queue_size=10,
        )
        self.on_transform_pub = rospy.Publisher(
            '/test_yaw_control/on/transform',
            HomogeneousTransform,
            queue_size=10,
        )
        self.subscribers = [
            rospy.Subscriber(
                '/test_yaw_control/off/cmd_vel_flu',
                TwistStamped,
                self._set_message,
                callback_args='off_body',
                queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/on/cmd_vel_flu',
                TwistStamped,
                self._set_message,
                callback_args='on_body',
                queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/off/preview',
                PositionTarget,
                self._set_message,
                callback_args='off_preview',
                queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/on/preview',
                PositionTarget,
                self._set_message,
                callback_args='on_preview',
                queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/off/error_flu_filtered',
                TwistStamped,
                self._set_message,
                callback_args='off_filtered',
                queue_size=100,
            ),
            rospy.Subscriber(
                '/test_yaw_control/off/status',
                String,
                self._set_status,
                callback_args='off_status',
                queue_size=10,
            ),
            rospy.Subscriber(
                '/test_yaw_control/on/status',
                String,
                self._set_status,
                callback_args='on_status',
                queue_size=10,
            ),
        ]
        self._wait_for(
            lambda: self.off_transform_pub.get_num_connections() > 0
            and self.on_transform_pub.get_num_connections() > 0,
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

    def _make_transform(self, body_error, stamp, yaw_error):
        target_camera = body_target_to_camera(
            [0.8, 0.0, 0.0],
            DEFAULT_BODY_R_CAMERA_OPTICAL,
            DEFAULT_BODY_T_CAMERA_OPTICAL,
        )
        camera_error = DEFAULT_BODY_R_CAMERA_OPTICAL.T.dot(
            np.asarray(body_error, dtype=np.float64)
        )
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

        transform = np.eye(4)
        transform[:3, :3] = np.array(
            [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]
        ).dot(FRONT_FACING_ROTATION)
        transform[:3, 3] = target_camera + camera_error

        message = HomogeneousTransform()
        message.header.stamp = stamp
        message.header.frame_id = 'usb_camera_optical_frame'
        message.marker_id = 33
        message.matrix = transform.reshape(16).tolist()
        return message

    def _publish_stage(self, body_error, yaw_error, duration):
        deadline = time.time() + duration
        while time.time() < deadline and not rospy.is_shutdown():
            stamp = rospy.Time.now()
            message = self._make_transform(body_error, stamp, yaw_error)
            self.off_transform_pub.publish(message)
            self.on_transform_pub.publish(message)
            rospy.sleep(0.05)

    def test_yaw_switch_outputs_and_alignment(self):
        self._publish_stage([0.3, 0.1, 0.1], 0.3, 0.5)
        self._wait_for(
            lambda: self.off_body is not None
            and self.on_body is not None
            and self.off_preview is not None
            and self.on_preview is not None
            and self.off_filtered is not None,
            2.0,
        )

        with self.lock:
            off_body = self.off_body
            on_body = self.on_body
            off_preview = self.off_preview
            on_preview = self.on_preview
            off_filtered = self.off_filtered

        self.assertGreater(abs(off_body.twist.linear.x), 0.0)
        self.assertGreater(abs(off_body.twist.linear.y), 0.0)
        self.assertGreater(abs(off_body.twist.linear.z), 0.0)
        self.assertEqual(off_body.twist.angular.z, 0.0)
        self.assertEqual(off_preview.yaw_rate, 0.0)
        self.assertTrue(math.isclose(off_filtered.twist.angular.z, 0.3))
        self.assertTrue(math.isclose(on_body.twist.angular.z, 0.12, abs_tol=0.01))
        self.assertTrue(math.isclose(on_preview.yaw_rate, 0.12, abs_tol=0.01))

        self._publish_stage([0.0, 0.0, 0.0], 0.3, 2.5)
        self._wait_for(
            lambda: 'ALIGNED' in self.off_status
            and 'yaw_control=OFF' in self.off_status
            and 'yaw_control=ON' in self.on_status,
            3.0,
        )
        with self.lock:
            off_status = self.off_status
            on_status = self.on_status
            off_preview = self.off_preview
            on_preview = self.on_preview

        self.assertIn('ALIGNED', off_status)
        self.assertIn('yaw_control=OFF', off_status)
        self.assertIn('ALIGN ID33', on_status)
        self.assertNotIn('ALIGNED ID33', on_status)
        self.assertIn('yaw_control=ON', on_status)
        self.assertEqual(off_preview.yaw_rate, 0.0)
        self.assertTrue(math.isclose(on_preview.yaw_rate, 0.12, abs_tol=0.01))

        published_topics = dict(rospy.get_published_topics())
        self.assertNotIn('/test_yaw_control/off/actual_setpoint', published_topics)
        self.assertNotIn('/test_yaw_control/on/actual_setpoint', published_topics)


if __name__ == '__main__':
    rospy.init_node('yaw_control_switch_test')
    rostest.rosrun(
        'front_marker_tracking',
        'yaw_control_switch',
        YawControlSwitchTest,
    )
