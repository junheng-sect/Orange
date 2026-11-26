#!/usr/bin/env python3
"""Node-level gating and interface checks with a real node and mocked ROS transport."""
import math
import unittest
from unittest import mock

import rospy
from mavros_msgs.msg import State, PositionTarget
from front_marker_control.control_node import FrontMarkerControlNode
from front_marker_control.refinement import DEFAULTS
from nav_msgs.msg import Odometry


def odom_message(stamp, position, yaw):
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = 'odom'
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
    message.pose.pose.orientation.z = math.sin(yaw/2)
    message.pose.pose.orientation.w = math.cos(yaw/2)
    return message


class RefinementNodeTest(unittest.TestCase):
    def node(self, yaw=True, position=True):
        params = {'~enable_setpoint_output': True, '~enable_yaw_control': yaw,
                  '~yaw_mode': 'marker', '~enable_position_hold': position}
        with mock.patch.object(rospy, 'get_param', side_effect=lambda key, default: params.get(key, default)), \
             mock.patch.object(rospy, 'Publisher', side_effect=lambda *a, **k: mock.MagicMock()), \
             mock.patch.object(rospy, 'Subscriber'), mock.patch.object(rospy, 'Timer'), \
             mock.patch('front_marker_control.control_node.tf2_ros.TransformListener'), \
             mock.patch('front_marker_control.control_node.tf2_ros.Buffer'), \
             mock.patch.object(rospy, 'logwarn'), mock.patch.object(rospy, 'loginfo'):
            return FrontMarkerControlNode()

    def test_feature_switches(self):
        self.assertIsNotNone(self.node().refiner)
        self.assertIsNone(self.node(yaw=False).refiner)
        self.assertIsNone(self.node(position=False).refiner)

    def test_local_velocity_mask_and_enu_values(self):
        node = self.node()
        message = node.make_local_velocity_setpoint(rospy.Time(100), (.1, -.2, .3), .4)
        self.assertEqual(message.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        self.assertEqual((message.velocity.x, message.velocity.y, message.velocity.z), (.1, -.2, .3))
        self.assertEqual(message.yaw, .4)
        for mask in (PositionTarget.IGNORE_PX, PositionTarget.IGNORE_PY, PositionTarget.IGNORE_PZ,
                     PositionTarget.IGNORE_AFX, PositionTarget.IGNORE_AFY, PositionTarget.IGNORE_AFZ,
                     PositionTarget.IGNORE_YAW_RATE):
            self.assertTrue(message.type_mask & mask)
        for mask in (PositionTarget.IGNORE_VX, PositionTarget.IGNORE_VY,
                     PositionTarget.IGNORE_VZ, PositionTarget.IGNORE_YAW):
            self.assertFalse(message.type_mask & mask)

    def test_sampling_survives_high_odom_speed(self):
        # 采样中速度超限：odom 回调不再重置采样。
        node = self.node()
        node.refiner.mode = 'YAW_SAMPLING'
        node.refiner.window_start = 100.0
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.05)):
            node.odom_callback(odom_message(rospy.Time.from_sec(100.05), (0, 0, 0), 0))
        self.assertEqual(node.refiner.mode, 'YAW_SAMPLING')
        self.assertEqual(node.refiner.window_start, 100.0)

    def test_odom_stream_gap_still_resets(self):
        # VIO 断流（接收间隔 > odom_timeout）：仍废弃采样窗口（硬门控）。
        node = self.node()
        node.refiner.mode = 'YAW_SAMPLING'
        node.refiner.window_start = 100.0
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time(100)):
            node.odom_callback(odom_message(rospy.Time(100), (0, 0, 0), 0))
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time.from_sec(100.3)):
            node.odom_callback(odom_message(rospy.Time.from_sec(100.3), (0, 0, 0), 0))
        self.assertEqual(node.refiner.mode, 'ALIGN')

    def test_transient_offboard_exit_resets_before_next_control_tick(self):
        node = self.node()
        node.refiner.mode = 'YAW_SAMPLING'
        node.refiner.window_start = 90.0
        node.refiner.samples = [(99, 0, (.8, 0, 0))]
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time(100)):
            node.state_callback(State(connected=True, armed=True, mode='MANUAL'))
            node.state_callback(State(connected=True, armed=True, mode='OFFBOARD'))
        self.assertEqual(node.refiner.mode, 'ALIGN')
        self.assertIsNone(node.refiner.window_start)
        self.assertFalse(node.refiner.samples)

    def test_pos_hold_vision_exit_disabled_with_refiner(self):
        # 精对准路径：POS_HOLD 期间视觉误差再大也不触发退出。
        node = self.node()
        node.refiner.mode = 'POS_HOLD'
        node.position_target = ((0, 0, 0), 0.0, 100.0)
        node.latest_position_error = (0.5, 0.2, 0.2)  # 远超退出带宽
        self.assertFalse(node.position_exit_exceeded())

    def test_pos_hold_vision_exit_kept_without_refiner(self):
        # 旧路径（yaw 关闭）：视觉误差超限退出仍生效。
        node = self.node(yaw=False)
        node.position_target = ((0, 0, 0), 0.0, 100.0)
        node.latest_position_error = (0.5, 0.0, 0.0)
        node.pos_exit_start_time = rospy.Time(100)
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time(101)):
            self.assertTrue(node.position_exit_exceeded())

    def test_yaw_disabled_position_lock_bypasses_sampling(self):
        node = self.node(yaw=False)
        with mock.patch.object(rospy.Time, 'now', return_value=rospy.Time(100)):
            node.odom_callback(odom_message(rospy.Time(100), (1, 2, 3), .4))
        node.latest_source_stamp = rospy.Time(100)
        node.latest_position_error = (0, 0, 0)
        node.latest_yaw_error = .2
        target = node.try_lock_position_target()
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target[1], .4)
        self.assertAlmostEqual(target[0][0], 1.)
        self.assertAlmostEqual(target[0][1], 2.)


if __name__ == '__main__':
    unittest.main()
