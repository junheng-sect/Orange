#!/usr/bin/env python3
"""Integration test: fixed_face ALIGN -> POS_HOLD with real topics and synthetic odom."""

import math
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
import rospy
import rostest
from std_msgs.msg import String

from front_marker_control.control import wrap_angle
from front_marker_control.mavros_interface import (
    position_only_type_mask,
    velocity_absolute_yaw_type_mask,
)

NODE = '/fixed_face_front_marker_control'


def camera_pose(body_error, stamp):
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = 'usb_camera_optical_frame'
    message.pose.position.x = -float(body_error[1])
    message.pose.position.y = -float(body_error[2])
    message.pose.position.z = 0.8 + float(body_error[0])
    message.pose.orientation.x = 1.0
    message.pose.orientation.w = 0.0
    return message


def odom_message(stamp, position, yaw, speed):
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = 'odom'
    message.pose.pose.position.x = float(position[0])
    message.pose.pose.position.y = float(position[1])
    message.pose.pose.position.z = float(position[2])
    message.pose.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.pose.orientation.w = math.cos(0.5 * yaw)
    message.twist.twist.linear.x = float(speed)
    return message


class FixedFaceIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.assertEqual(rospy.get_param(NODE + '/yaw_mode'), 'fixed_face')
        self.actual = []
        self.statuses = []
        self.modes = []
        self.pose_pub = rospy.Publisher('/test_fixed_face/pose', PoseStamped, queue_size=10)
        self.state_pub = rospy.Publisher('/test_fixed_face/state', State, queue_size=10)
        self.odom_pub = rospy.Publisher('/test_fixed_face/odom', Odometry, queue_size=10)
        self.subscribers = [
            rospy.Subscriber('/test_fixed_face/actual', PositionTarget, self._append,
                             callback_args=self.actual, queue_size=200),
            rospy.Subscriber('/test_fixed_face/status', String, self._append,
                             callback_args=self.statuses, queue_size=200),
            rospy.Subscriber('/test_fixed_face/control_mode', String, self._append,
                             callback_args=self.modes, queue_size=50),
        ]
        self.wait_until(
            lambda: self.pose_pub.get_num_connections() > 0
            and self.state_pub.get_num_connections() > 0
            and self.odom_pub.get_num_connections() > 0,
            5.0, 'publisher connections')
        # 每个用例前退出 OFFBOARD：释放货架朝向、回到 ALIGN。
        self.drive(0.4, mode='POSCTL')
        with self.lock:
            del self.actual[:]
            del self.statuses[:]
            del self.modes[:]

    def tearDown(self):
        for subscriber in self.subscribers:
            subscriber.unregister()

    def _append(self, message, target):
        with self.lock:
            target.append(message)

    def wait_until(self, predicate, timeout, description):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail('%s not met within %.1fs' % (description, timeout))

    def drive(self, duration, mode='OFFBOARD', yaw=0.0, speed=0.0,
              position=(0.0, 0.0, 1.2), body_error=(0.0, 0.0, 0.0), until=None):
        """Keep state/odom/pose fresh at ~30 Hz; stop early when ``until`` holds."""
        deadline = time.time() + duration
        while time.time() < deadline and not rospy.is_shutdown():
            state = State()
            state.connected = True
            state.armed = True
            state.mode = mode
            self.state_pub.publish(state)
            now = rospy.Time.now()
            self.odom_pub.publish(odom_message(now, position, yaw, speed))
            self.pose_pub.publish(camera_pose(body_error, now))
            if until is not None and until():
                return True
            rospy.sleep(0.033)
        return False

    def latest(self, items):
        with self.lock:
            return items[-1] if items else None

    def mode_seen(self, name):
        with self.lock:
            return any(message.data == name for message in self.modes)

    def test_align_locks_face_yaw_then_holds(self):
        yaw = math.radians(3.0)
        reached = self.drive(6.0, yaw=yaw, until=lambda: self.mode_seen('POS_HOLD'))
        self.assertTrue(reached, 'POS_HOLD not reached')
        with self.lock:
            align = [m for m in self.actual
                     if m.type_mask == velocity_absolute_yaw_type_mask()]
        self.assertTrue(align, 'ALIGN did not publish local velocity + absolute yaw')
        self.assertEqual(align[0].coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        # yaw 设定从当前航向（3°）斜坡转向货架朝向 0°。
        self.assertLessEqual(align[0].yaw, yaw + 1e-6)
        self.assertGreater(align[0].yaw, 0.0)
        self.wait_until(
            lambda: (self.latest(self.actual) is not None
                     and self.latest(self.actual).type_mask == position_only_type_mask()),
            1.0, 'position setpoint')
        hold = self.latest(self.actual)
        self.assertAlmostEqual(hold.yaw, 0.0, places=6)
        # Marker 在机体正前 0.8 m（机体 yaw 3°），悬停点按货架朝向 0° 往回退 0.8 m。
        self.assertAlmostEqual(hold.position.x, 0.8 * math.cos(yaw) - 0.8, places=3)
        self.assertAlmostEqual(hold.position.y, 0.8 * math.sin(yaw), places=3)
        self.assertAlmostEqual(hold.position.z, 1.2, places=3)

    def test_large_heading_offset_waits_without_moving(self):
        self.drive(1.0, yaw=math.radians(40.0))
        self.assertFalse(self.mode_seen('POS_HOLD'))
        status = self.latest(self.statuses)
        self.assertIsNotNone(status)
        self.assertIn('WAIT_FACE_YAW', status.data)
        message = self.latest(self.actual)
        self.assertEqual(message.coordinate_frame, PositionTarget.FRAME_BODY_NED)
        self.assertEqual((message.velocity.x, message.velocity.y, message.velocity.z),
                         (0.0, 0.0, 0.0))

    def test_moving_vehicle_waits_still(self):
        # status 只在运动状态变化时立即发布，WAIT_STILL 靠 1 Hz 周期补发，需跨过一个周期。
        self.drive(2.5, yaw=0.0, speed=0.2)
        self.assertFalse(self.mode_seen('POS_HOLD'))
        with self.lock:
            texts = [message.data for message in self.statuses]
        self.assertTrue(any('WAIT_STILL' in text for text in texts), texts[-3:])
        reached = self.drive(3.0, yaw=0.0, speed=0.0, until=lambda: self.mode_seen('POS_HOLD'))
        self.assertTrue(reached, 'POS_HOLD not reached after the vehicle stopped')

    def test_opposite_face_selected(self):
        yaw = math.radians(175.0)
        self.drive(0.5, yaw=yaw)
        with self.lock:
            align = [m for m in self.actual
                     if m.type_mask == velocity_absolute_yaw_type_mask()]
        self.assertTrue(align)
        # 朝 180° 方向转：yaw 设定比 175° 更接近 ±180°。
        self.assertLess(abs(wrap_angle(align[-1].yaw - math.pi)),
                        abs(wrap_angle(yaw - math.pi)))


if __name__ == '__main__':
    rospy.init_node('test_fixed_face_integration')
    rostest.rosrun('front_marker_control', 'fixed_face', FixedFaceIntegrationTest)
