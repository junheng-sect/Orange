#!/usr/bin/env python3
"""fixed_face yaw mode: shelf heading latch, yaw ramp, stillness gate, median lock."""
import math
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry

from front_marker_control.control import wrap_angle
from front_marker_control.control_node import FrontMarkerControlNode
from front_marker_control.mavros_interface import velocity_absolute_yaw_type_mask


def odom_message(stamp, position, yaw, speed=0.0):
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = 'odom'
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
    message.pose.pose.orientation.z = math.sin(yaw / 2)
    message.pose.pose.orientation.w = math.cos(yaw / 2)
    message.twist.twist.linear.x = speed
    return message


def state_message(mode='OFFBOARD'):
    message = State()
    message.connected = True
    message.armed = True
    message.mode = mode
    return message


class FixedFaceTest(unittest.TestCase):
    def node(self, **overrides):
        params = {'~enable_setpoint_output': True, '~enable_position_hold': True}
        params.update({'~' + key: value for key, value in overrides.items()})
        with mock.patch.object(rospy, 'get_param', side_effect=lambda key, default: params.get(key, default)), \
             mock.patch.object(rospy, 'Publisher', side_effect=lambda *a, **k: mock.MagicMock()), \
             mock.patch.object(rospy, 'Subscriber'), mock.patch.object(rospy, 'Timer'), \
             mock.patch('front_marker_control.control_node.tf2_ros.TransformListener'), \
             mock.patch('front_marker_control.control_node.tf2_ros.Buffer'), \
             mock.patch.object(rospy, 'logwarn'), mock.patch.object(rospy, 'loginfo'):
            return FrontMarkerControlNode()

    @staticmethod
    def at(seconds):
        return mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(seconds))

    def feed_odom(self, node, seconds, position=(0.0, 0.0, 0.0), yaw=0.0, speed=0.0):
        with self.at(seconds):
            node.odom_callback(odom_message(rospy.Time.from_sec(seconds), position, yaw, speed))

    # ---- parameters -------------------------------------------------------
    def test_default_is_fixed_face_without_refiner(self):
        node = self.node()
        self.assertEqual(node.yaw_mode, 'fixed_face')
        self.assertIsNone(node.refiner)
        self.assertFalse(node.enable_yaw_control)
        self.assertEqual(node.yaw_control_label(), 'FACE')

    def test_mode_switches(self):
        self.assertIsNotNone(self.node(yaw_mode='marker').refiner)
        self.assertEqual(self.node(enable_yaw_control=False).yaw_mode, 'off')
        with self.assertRaises(ValueError):
            self.node(yaw_mode='tag')
        with self.assertRaises(ValueError):
            self.node(enable_position_hold=False)

    # ---- heading latch and ramp ---------------------------------------------
    def test_latches_nearest_face_and_ramps(self):
        node = self.node()
        self.feed_odom(node, 100.0, yaw=math.radians(10.0))
        self.assertTrue(node.update_face_yaw(0.05))
        self.assertEqual(node.face_yaw, 0.0)
        self.assertAlmostEqual(math.degrees(node.face_yaw_command), 9.5, places=6)
        for _ in range(19):
            self.assertFalse(node.update_face_yaw(0.05))
        self.assertEqual(node.face_yaw_command, node.face_yaw)

    def test_opposite_face_wraps(self):
        node = self.node()
        self.feed_odom(node, 100.0, yaw=math.radians(-172.0))
        node.update_face_yaw(0.05)
        self.assertAlmostEqual(abs(wrap_angle(node.face_yaw - math.pi)), 0.0, places=9)
        # 走圆周最短方向：-172° 往 -180° 转，而不是绕回 +180° 一大圈。
        self.assertAlmostEqual(math.degrees(node.face_yaw_command), -172.5, places=6)

    def test_large_offset_refuses_to_latch(self):
        node = self.node()
        self.feed_odom(node, 100.0, yaw=math.radians(40.0))
        self.assertFalse(node.update_face_yaw(0.05))
        self.assertIsNone(node.face_yaw)
        self.assertIn('航向偏离', node.face_status())

    def test_offboard_exit_and_localization_reject_release_face(self):
        node = self.node()
        self.feed_odom(node, 100.0)
        node.update_face_yaw(0.05)
        with self.at(100.0):
            node.state_callback(state_message('POSCTL'))
        self.assertIsNone(node.face_yaw)
        node.update_face_yaw(0.05)
        node._localization_rejected('test')
        self.assertIsNone(node.face_yaw)

    # ---- alignment gates ------------------------------------------------------
    def aligned_node(self, body_yaw, speed):
        node = self.node(aligned_hold_sec=0.2)
        node.face_yaw = node.face_yaw_command = 0.0
        self.feed_odom(node, 100.0, yaw=body_yaw, speed=speed)
        node.latest_position_error = (0.0, 0.0, 0.0)
        node.latest_source_stamp = rospy.Time.from_sec(100.0)
        return node

    def test_stillness_gate(self):
        node = self.aligned_node(0.0, speed=0.2)
        self.assertEqual(node.update_alignment_state(rospy.Time.from_sec(100.0)), 'ALIGN')
        self.assertEqual(node.update_alignment_state(rospy.Time.from_sec(100.3)), 'WAIT_STILL')
        self.feed_odom(node, 100.31, speed=0.01)
        self.assertEqual(node.update_alignment_state(rospy.Time.from_sec(100.32)), 'ALIGNED')

    def test_unsettled_yaw_blocks_alignment(self):
        node = self.aligned_node(math.radians(8.0), speed=0.0)
        node.update_alignment_state(rospy.Time.from_sec(100.0))
        self.assertEqual(node.update_alignment_state(rospy.Time.from_sec(100.3)), 'ALIGN')
        self.assertIsNone(node.aligned_start_time)

    # ---- median lock ------------------------------------------------------------
    def test_lock_uses_window_median_and_face_yaw(self):
        node = self.node()
        yaw = 0.03
        for index in range(15):
            self.feed_odom(node, 100.0 + 0.05 * index, position=(1.0, 2.0, 1.2), yaw=yaw)
        node.face_yaw = node.face_yaw_command = 0.0
        node.aligned_start_stamp = 100.1
        node.recent_markers.append((100.0, (5.0, 5.0, 5.0)))  # 早于对准窗口，不参与
        bodies = [(0.80, 0.0, 0.0)] * 6 + [(0.90, 0.05, -0.04)]
        for index, body in enumerate(bodies):
            node.recent_markers.append((100.12 + 0.07 * index, body))
        p_target, yaw_target, stamp = node.try_lock_face_target()
        self.assertEqual(yaw_target, 0.0)
        self.assertAlmostEqual(stamp, 100.12 + 0.07 * 6)
        expected = (1.0 + 0.8 * math.cos(yaw) - 0.8, 2.0 + 0.8 * math.sin(yaw), 1.2)
        for got, want in zip(p_target, expected):
            self.assertAlmostEqual(got, want, places=6)

    def test_lock_needs_enough_samples(self):
        node = self.node()
        for index in range(5):
            self.feed_odom(node, 100.0 + 0.05 * index)
        node.face_yaw = node.face_yaw_command = 0.0
        node.aligned_start_stamp = 100.0
        node.recent_markers.extend([(100.05, (0.8, 0.0, 0.0)), (100.1, (0.8, 0.0, 0.0))])
        self.assertIsNone(node.try_lock_face_target())
        self.assertIn('锁定样本不足', node.last_localization_issue)

    # ---- output ----------------------------------------------------------------
    def test_align_publishes_local_velocity_with_ramped_yaw(self):
        node = self.node()
        with self.at(100.0):
            node.state_callback(state_message())
        self.feed_odom(node, 100.0, yaw=math.radians(90.0 + 10.0))
        node.face_yaws = (math.radians(90.0),)
        node.latest_position_error = (0.2, 0.0, 0.0)
        node.latest_source_stamp = rospy.Time.from_sec(100.0)
        node.latest_receive_time = rospy.Time.from_sec(100.0)
        node.latest_body_velocity = (0.1, 0.0, 0.0)
        with self.at(100.02):
            node.control_loop(None)
        message = node.setpoint_pub.publish.call_args[0][0]
        self.assertEqual(message.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        self.assertEqual(message.type_mask, velocity_absolute_yaw_type_mask())
        # PositionTarget 字段为 float32，比较放宽到 1e-4 deg。
        self.assertAlmostEqual(math.degrees(message.yaw), 99.5, places=4)
        # 机体前向速度按 100° 航向转到 local ENU。
        speed = math.hypot(message.velocity.x, message.velocity.y)
        self.assertGreater(speed, 0.0)
        self.assertAlmostEqual(
            math.degrees(math.atan2(message.velocity.y, message.velocity.x)), 100.0, places=4)

    def test_large_offset_outputs_zero_body_velocity(self):
        node = self.node()
        with self.at(100.0):
            node.state_callback(state_message())
        self.feed_odom(node, 100.0, yaw=math.radians(45.0))
        node.latest_position_error = (0.2, 0.0, 0.0)
        node.latest_source_stamp = rospy.Time.from_sec(100.0)
        node.latest_receive_time = rospy.Time.from_sec(100.0)
        node.latest_body_velocity = (0.1, 0.0, 0.0)
        with self.at(100.02):
            node.control_loop(None)
        message = node.setpoint_pub.publish.call_args[0][0]
        self.assertEqual(message.coordinate_frame, PositionTarget.FRAME_BODY_NED)
        self.assertEqual((message.velocity.x, message.velocity.y, message.velocity.z), (0.0, 0.0, 0.0))
        self.assertTrue(node.last_status.startswith('WAIT_FACE_YAW | actual=zero'))


if __name__ == '__main__':
    unittest.main()
