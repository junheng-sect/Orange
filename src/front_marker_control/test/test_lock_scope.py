#!/usr/bin/env python3

import math
import threading
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rospy

from front_marker_control.control_node import FrontMarkerControlNode
from front_marker_control.localization import LocalPoseBuffer


class RecordingPublisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class BlockingCall(object):
    def __init__(self, result=None):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.result = result

    def wait(self):
        self.entered.set()
        if not self.release.wait(2.0):
            raise RuntimeError('blocking test call was not released')
        return self.result


class BlockingPublisher(BlockingCall):
    def publish(self, _message):
        return self.wait()


class RosExceptionPublisher(object):
    def __init__(self, message='publish() to a closed topic'):
        self.message = message

    def publish(self, _message):
        raise rospy.ROSException(self.message)


class BlockingTransform(BlockingCall):
    def transform(self, _message, _target_frame, _timeout):
        return self.wait()


class ImmediateTransform(object):
    def __init__(self, result):
        self.result = result

    def transform(self, _message, _target_frame, _timeout):
        return self.result


class FakeBodyController(object):
    def reset(self):
        pass

    def update(self, _error, _yaw_error, _dt):
        return (0.1, 0.0, 0.0), 0.0, 0.0


class FakeVelocityLimiter(object):
    def reset(self):
        return (0.0, 0.0, 0.0)

    def update(self, target, _rate_limit, _dt):
        return tuple(target)


def pose_at(stamp, x=0.8):
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = 'usb_camera_optical_frame'
    message.pose.position.x = x
    # Body-frame Marker normal points toward -X, so heading is well-defined.
    message.pose.orientation.y = -0.7071067811865476
    message.pose.orientation.w = 0.7071067811865476
    return message


class LockScopeTest(unittest.TestCase):
    def setUp(self):
        self.now = rospy.Time.from_sec(100.0)

    def pose_node(self, transform, position_publisher=None):
        node = FrontMarkerControlNode.__new__(FrontMarkerControlNode)
        node.refiner = None
        node.lock = threading.RLock()
        node.last_accepted_source_stamp = None
        node.last_filter_stamp = None
        node.max_pose_latency_sec = 0.5
        node.future_stamp_tolerance_sec = 0.1
        node.tf_buffer = transform
        node.base_frame = 'base_link'
        node.tf_lookup_timeout_sec = 0.05
        node.target_xyz_body = (0.8, 0.0, 0.0)
        node.setpoint_publish_rate_hz = 20.0
        node.filter_dt_max_sec = 0.5
        node.body_controller = FakeBodyController()
        node.latest_position_error = None
        node.latest_yaw_error = 0.0
        node.latest_filtered_yaw_error = 0.0
        node.latest_source_stamp = None
        node.latest_receive_time = None
        node.latest_body_velocity = (0.0, 0.0, 0.0)
        node.latest_yaw_rate = 0.0
        node.aligned_start_time = None
        node.last_observation_issue = ''
        node.enable_position_hold = False
        node.local_pose_buffer = LocalPoseBuffer()
        node.latest_odom_receive_time = None
        node.position_error_pub = position_publisher or RecordingPublisher()
        node.yaw_error_pub = RecordingPublisher()
        return node

    def control_node(self, command_publisher=None, status_publisher=None):
        node = FrontMarkerControlNode.__new__(FrontMarkerControlNode)
        node.refiner = None
        node.lock = threading.RLock()
        node.setpoint_publish_rate_hz = 20.0
        node.last_control_time = None
        node.latest_position_error = (0.2, 0.0, 0.0)
        node.latest_yaw_error = 0.0
        node.latest_filtered_yaw_error = 0.0
        node.latest_source_stamp = self.now
        node.latest_receive_time = self.now
        node.pose_timeout_sec = 0.1
        node.latest_body_velocity = (0.1, 0.0, 0.0)
        node.latest_yaw_rate = 0.0
        node.update_alignment_state = lambda _now: 'ALIGN'
        node.enable_setpoint_output = False
        node.velocity_limiter = FakeVelocityLimiter()
        node.max_accel = 0.5
        node.loss_decel = 1.0
        node.last_observation_issue = ''
        node.enable_yaw_control = False
        node.base_frame = 'base_link'
        node.enable_position_hold = False
        node.control_mode = 'ALIGN'
        node.mode_reason = ''
        node.position_target = None
        node.pos_exit_start_time = None
        node.aligned_start_time = None
        node.local_pose_buffer = LocalPoseBuffer()
        node.latest_odom_receive_time = None
        node.last_localization_issue = ''
        node.last_odom_issue = ''
        node.command_pub = command_publisher or RecordingPublisher()
        node.preview_pub = RecordingPublisher()
        node.setpoint_pub = None
        node.last_status = ''
        node.last_motion_state = None
        node.last_control_mode = None
        node.last_mode_reason = None
        node.status_pub = status_publisher or RecordingPublisher()
        return node

    def assert_lock_available_during_call(self, node, call, target):
        errors = []

        def run():
            try:
                target()
            except BaseException as exception:  # Propagate worker failures.
                errors.append(exception)

        worker = threading.Thread(target=run)
        worker.start()
        reached_call = call.entered.wait(1.0)
        if not reached_call:
            call.release.set()
            worker.join(1.0)
            if errors:
                raise errors[0]
            self.fail('worker did not reach blocking call')
        acquired = False
        try:
            acquired = node.lock.acquire(timeout=0.2)
            self.assertTrue(acquired, 'shared state lock was held by blocking I/O')
        finally:
            if acquired:
                node.lock.release()
            call.release.set()
            worker.join(1.0)
        self.assertFalse(worker.is_alive(), 'worker did not finish')
        if errors:
            raise errors[0]

    def test_tf_lookup_does_not_hold_shared_state_lock(self):
        transformed = pose_at(self.now)
        transformed.header.frame_id = 'base_link'
        blocking_transform = BlockingTransform(transformed)
        node = self.pose_node(blocking_transform)
        source = pose_at(self.now)
        with mock.patch.object(rospy.Time, 'now', return_value=self.now):
            self.assert_lock_available_during_call(
                node,
                blocking_transform,
                lambda: node.pose_callback(source),
            )

    def test_error_publish_does_not_hold_shared_state_lock(self):
        transformed = pose_at(self.now)
        transformed.header.frame_id = 'base_link'
        blocking_publisher = BlockingPublisher()
        node = self.pose_node(
            ImmediateTransform(transformed),
            position_publisher=blocking_publisher,
        )
        source = pose_at(self.now)
        with mock.patch.object(rospy.Time, 'now', return_value=self.now):
            self.assert_lock_available_during_call(
                node,
                blocking_publisher,
                lambda: node.pose_callback(source),
            )

    def test_command_publish_does_not_hold_shared_state_lock(self):
        blocking_publisher = BlockingPublisher()
        node = self.control_node(command_publisher=blocking_publisher)
        with mock.patch.object(rospy.Time, 'now', return_value=self.now):
            self.assert_lock_available_during_call(
                node,
                blocking_publisher,
                lambda: node.control_loop(None),
            )

    def test_status_publish_does_not_hold_shared_state_lock(self):
        blocking_publisher = BlockingPublisher()
        node = self.control_node(status_publisher=blocking_publisher)
        with mock.patch.object(rospy.Time, 'now', return_value=self.now):
            self.assert_lock_available_during_call(
                node,
                blocking_publisher,
                lambda: node.control_loop(None),
            )

    def test_shutdown_publish_race_is_ignored(self):
        publisher = RosExceptionPublisher()
        with mock.patch.object(rospy, 'is_shutdown', return_value=False):
            self.assertFalse(
                FrontMarkerControlNode._publish_if_running(publisher, object())
            )

    def test_non_shutdown_publish_failure_is_not_hidden(self):
        publisher = RosExceptionPublisher('unexpected transport failure')
        with mock.patch.object(rospy, 'is_shutdown', return_value=False):
            with self.assertRaises(rospy.ROSException):
                FrontMarkerControlNode._publish_if_running(publisher, object())


class PositionHoldRegressionTest(unittest.TestCase):
    """Coverage for the ALIGN/POS_HOLD/RETREAT + localization-outage fixes."""

    def setUp(self):
        self.now = rospy.Time.from_sec(100.0)

    def hold_node(self, control_mode='ALIGN', error_x=0.0):
        """A hold-enabled node whose localization never arrived (stale)."""
        node = FrontMarkerControlNode.__new__(FrontMarkerControlNode)
        node.refiner = None
        node.lock = threading.RLock()
        node.setpoint_publish_rate_hz = 20.0
        node.last_control_time = None
        node.latest_position_error = (error_x, 0.0, 0.0)
        node.latest_yaw_error = 0.0
        node.latest_filtered_yaw_error = 0.0
        node.latest_source_stamp = self.now
        node.latest_receive_time = self.now
        node.pose_timeout_sec = 0.1
        node.latest_body_velocity = (0.1, 0.0, 0.0)
        node.latest_yaw_rate = 0.0
        node.update_alignment_state = lambda _now: 'ALIGN'
        node.enable_setpoint_output = False
        node.enable_yaw_control = False
        node.enable_position_hold = True
        node.base_frame = 'base_link'
        node.velocity_limiter = FakeVelocityLimiter()
        node.max_accel = 0.5
        node.loss_decel = 1.0
        node.last_observation_issue = ''
        node.control_mode = control_mode
        node.mode_reason = ''
        node.position_target = None
        node.pos_exit_start_time = None
        node.aligned_start_time = None
        node.local_pose_buffer = LocalPoseBuffer()
        node.latest_odom_receive_time = None  # localization stale/never arrived
        node.last_localization_issue = ''
        node.last_odom_issue = ''
        node.target_xyz_body = (0.8, 0.0, 0.0)
        node.min_marker_distance_m = 0.6
        node.retreat_recover_distance_m = 0.7
        node.retreat_velocity_x = -0.1
        node.pos_exit_distance_tolerance_m = 0.18
        node.pos_exit_lateral_tolerance_m = 0.12
        node.pos_exit_vertical_tolerance_m = 0.12
        node.pos_exit_yaw_tolerance_rad = math.radians(9.0)
        node.pos_exit_hold_sec = 1.0
        node.command_pub = RecordingPublisher()
        node.preview_pub = RecordingPublisher()
        node.setpoint_pub = None
        node.last_status = ''
        node.last_motion_state = None
        node.last_control_mode = None
        node.last_mode_reason = None
        node.status_pub = RecordingPublisher()
        return node

    def run_control_cycle(self, node):
        with mock.patch.object(rospy.Time, 'now', return_value=self.now), \
                mock.patch.object(rospy, 'is_shutdown', return_value=False), \
                mock.patch.object(rospy, 'logwarn_throttle'):
            node.control_loop(None)

    def test_retreat_velocity_survives_localization_loss(self):
        """RETREAT must not be overridden to zero by stale localization."""
        # marker_x = -0.25 + 0.8 = 0.55 m (near wall, still inside retreat).
        node = self.hold_node(control_mode='RETREAT', error_x=-0.25)
        self.run_control_cycle(node)
        self.assertEqual('RETREAT', node.control_mode)
        command = node.command_pub.messages[-1]
        self.assertEqual(-0.1, command.twist.linear.x)

    def test_near_wall_retreat_takes_effect_same_cycle(self):
        """Near-wall RETREAT is decided before command build: the trigger cycle
        must already command a clean backward-only velocity, never a leftover
        forward/lateral tracking command."""
        node = self.hold_node(control_mode='ALIGN', error_x=-0.25)
        self.run_control_cycle(node)
        self.assertEqual('RETREAT', node.control_mode)
        command = node.command_pub.messages[-1]
        self.assertLess(command.twist.linear.x, 0.0)
        self.assertEqual(0.0, command.twist.linear.y)
        self.assertEqual(0.0, command.twist.linear.z)
        self.assertEqual(0.0, command.twist.angular.z)
        # 若仍在 ALIGN，则说明触发周期没完成切换（回归）。
        self.run_control_cycle(node)
        self.assertEqual('RETREAT', node.control_mode)
        command = node.command_pub.messages[-1]
        self.assertLessEqual(command.twist.linear.x, 0.0)

    def test_near_wall_threshold_boundaries(self):
        """min_marker_distance_m=0.50 的触发/恢复边界：
        触发用严格小于门限，恢复用大于等于恢复距离。
        marker_x = error_x + target_xyz_body[0]（0.8）。"""
        def make_node(control_mode, error_x):
            node = self.hold_node(control_mode=control_mode, error_x=error_x)
            node.min_marker_distance_m = 0.5
            node.retreat_recover_distance_m = 0.7
            return node

        # x = 0.49 m：当周期触发后退。
        node = make_node('ALIGN', -0.31)
        self.run_control_cycle(node)
        self.assertEqual('RETREAT', node.control_mode)

        # x = 0.50 / 0.51 m：不触发（严格小于门限）。
        for error_x in (-0.30, -0.29):
            node = make_node('ALIGN', error_x)
            self.run_control_cycle(node)
            self.assertEqual('ALIGN', node.control_mode)

        # 后退中 x = 0.69 m：未达恢复距离，继续后退。
        node = make_node('RETREAT', -0.11)
        self.run_control_cycle(node)
        self.assertEqual('RETREAT', node.control_mode)
        self.assertLess(node.command_pub.messages[-1].twist.linear.x, 0.0)

        # 后退中 x = 0.70 m：达到恢复距离，重新对准。
        node = make_node('RETREAT', -0.10)
        self.run_control_cycle(node)
        self.assertEqual('ALIGN', node.control_mode)

    def test_localization_loss_is_reported(self):
        """A stale/unavailable localization surfaces a reason, not silence."""
        node = self.hold_node(control_mode='ALIGN', error_x=0.0)
        with mock.patch.object(rospy.Time, 'now', return_value=self.now), \
                mock.patch.object(rospy, 'is_shutdown', return_value=False):
            warnings = []
            with mock.patch.object(
                rospy, 'logwarn_throttle', side_effect=(
                    lambda *args: warnings.append(args)
                ),
            ):
                node.control_loop(None)
        self.assertTrue(node.last_localization_issue)
        self.assertIn('定位尚未收到 Odometry', node.last_localization_issue)
        self.assertTrue(warnings, 'expected a throttled localization warning')
        self.assertEqual('ALIGN', node.control_mode)
        self.assertEqual(0.0, node.command_pub.messages[-1].twist.linear.x)


class LocalizationRejectRegressionTest(unittest.TestCase):
    """odom 源/接收时间校验与拒绝时的原子清除。"""

    def setUp(self):
        self.now = rospy.Time.from_sec(100.0)

    def hold_odom_node(self, control_mode='POS_HOLD'):
        node = FrontMarkerControlNode.__new__(FrontMarkerControlNode)
        node.refiner = None
        node.lock = threading.RLock()
        node.odom_source_max_age_sec = 0.5
        node.odom_interp_gap_max_sec = 0.1
        node.future_stamp_tolerance_sec = 0.1
        node.local_pose_buffer = LocalPoseBuffer()
        node.control_mode = control_mode
        node.mode_reason = ''
        node.position_target = ((0.0, 0.0, 0.0), 0.0, 100.0)
        node.pos_exit_start_time = None
        node.aligned_start_time = None
        node.latest_odom_receive_time = self.now
        node.latest_odom_source_stamp = self.now
        node.last_odom_issue = ''
        node.last_localization_issue = ''
        return node

    def odom_message(self, stamp, frame='odom', position=(0.0, 0.0, 0.0)):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = 0.0
        message.pose.pose.orientation.w = 1.0
        return message

    def run_odom(self, node, message):
        with mock.patch.object(rospy.Time, 'now', return_value=self.now):
            node.odom_callback(message)

    def test_stale_source_odom_clears_position_target(self):
        """99s 旧的定位源必须判为不可用并立即清掉 POS_HOLD 目标。"""
        node = self.hold_odom_node()
        stale = self.now.to_sec() - 99.0
        odom = self.odom_message(rospy.Time.from_sec(stale))
        self.run_odom(node, odom)
        self.assertIsNone(node.latest_odom_receive_time)
        self.assertIsNone(node.position_target)
        self.assertEqual('ALIGN', node.control_mode)
        self.assertIn('过期', node.last_odom_issue)

    def test_reject_then_valid_does_not_resurrect_target(self):
        """非法 odom 拒绝后在回调内即清除；随后立即到达的有效帧不得复活旧目标。"""
        node = self.hold_odom_node()
        bad = self.odom_message(self.now, position=(float('nan'), 0.0, 0.0))
        self.run_odom(node, bad)
        self.assertIsNone(node.position_target)
        self.assertEqual('ALIGN', node.control_mode)
        # “非法 → 有效”紧邻：若清除被推迟到控制循环就会被错过。
        good = self.odom_message(self.now)
        self.run_odom(node, good)
        self.assertIsNotNone(node.latest_odom_receive_time)
        self.assertEqual('ALIGN', node.control_mode)
        self.assertIsNone(node.position_target)

    def test_odom_reject_preserves_retreat(self):
        """定位异常不打断纯视觉的近墙 RETREAT。"""
        node = self.hold_odom_node(control_mode='RETREAT')
        node.position_target = None
        odom = self.odom_message(
            rospy.Time.from_sec(self.now.to_sec() - 99.0)
        )
        self.run_odom(node, odom)
        self.assertEqual('RETREAT', node.control_mode)

    def test_pos_hold_exit_timer_resets_on_pose_loss(self):
        """POS_HOLD 丢标期间退出计时清零：恢复首帧不能凭旧计时立刻退出。"""
        node = self.hold_odom_node()
        node.pos_exit_start_time = rospy.Time.from_sec(self.now.to_sec() - 5.0)
        with mock.patch.object(rospy.Time, 'now', return_value=self.now):
            node.update_control_mode(self.now, False, None, False)
        self.assertEqual('POS_HOLD', node.control_mode)
        self.assertIsNone(node.pos_exit_start_time)


if __name__ == '__main__':
    unittest.main()
