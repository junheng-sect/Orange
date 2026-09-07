#!/usr/bin/env python3
"""Integration test: ALIGN -> POS_HOLD lock, retreat, gates, localization."""

import math
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
import rospy
import rostest
from std_msgs.msg import String

from front_marker_control.control import wrap_angle


def camera_pose(body_error, stamp, yaw_error=0.0):
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = 'usb_camera_optical_frame'
    message.pose.position.x = -float(body_error[1])
    message.pose.position.y = -float(body_error[2])
    message.pose.position.z = 0.8 + float(body_error[0])
    # marker normal +Y in body FLU -> yaw_error is heading of -X axis.
    message.pose.orientation.x = math.cos(yaw_error * 0.5)
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = math.sin(yaw_error * 0.5)
    message.pose.orientation.w = 0.0
    return message


def odom_message(stamp, position, yaw, frame='odom'):
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = frame
    message.pose.pose.position.x = float(position[0])
    message.pose.pose.position.y = float(position[1])
    message.pose.pose.position.z = float(position[2])
    message.pose.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.pose.orientation.w = math.cos(0.5 * yaw)
    return message


def odom_to_rospy_publish(odom_pub, position, yaw, frame='odom', stamp=None):
    stamp = stamp or rospy.Time.now()
    message = odom_message(stamp, position, yaw, frame)
    odom_pub.publish(message)
    return stamp


class PositionHoldTest(unittest.TestCase):
    aligned_error = (0.0, 0.0, 0.0)
    far_error = (0.5, 0.0, 0.0)

    def setUp(self):
        self.lock = threading.Lock()
        self.commands = []
        self.previews = []
        self.actual = []
        self.statuses = []
        self.pose_pub = rospy.Publisher(
            '/test_pos_hold/pose', PoseStamped, queue_size=10
        )
        self.state_pub = rospy.Publisher(
            '/test_pos_hold/state', State, queue_size=10
        )
        self.odom_pub = rospy.Publisher(
            '/test_pos_hold/odom', Odometry, queue_size=10
        )
        rospy.Subscriber(
            '/test_pos_hold/cmd_vel', TwistStamped,
            self._append, callback_args=self.commands, queue_size=100,
        )
        rospy.Subscriber(
            '/test_pos_hold/preview', PositionTarget,
            self._append, callback_args=self.previews, queue_size=100,
        )
        rospy.Subscriber(
            '/test_pos_hold/actual', PositionTarget,
            self._append, callback_args=self.actual, queue_size=100,
        )
        rospy.Subscriber(
            '/test_pos_hold/status', String,
            self._append, callback_args=self.statuses, queue_size=200,
        )
        self.wait_for(
            lambda: self.pose_pub.get_num_connections() > 0
            and self.state_pub.get_num_connections() > 0
            and self.odom_pub.get_num_connections() > 0,
            5.0,
        )

    def _append(self, message, target):
        with self.lock:
            target.append(message)

    def wait_for(self, predicate, timeout, description='condition'):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail('%s not met within %.1fs' % (description, timeout))

    def publish_state(self, connected=True, armed=True, mode='OFFBOARD'):
        message = State()
        message.connected = connected
        message.armed = armed
        message.mode = mode
        self.state_pub.publish(message)

    def publish_and_wait(
        self, predicate, timeout, description,
        body_error=None, yaw_error=0.0,
        odom_position=(0.0, 0.0, 0.0), odom_yaw=0.0, publish_odom=True,
        state_mode='OFFBOARD', state_connected=True, state_armed=True,
    ):
        """持续发布输入的同时等待条件成立。

        等待期间必须保持 state/odom/pose 新鲜，否则节点会因门控或
        定位超时不断重置对齐计时，条件永远无法满足。
        """
        if body_error is None:
            body_error = self.aligned_error
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self.publish_state(state_connected, state_armed, state_mode)
            if publish_odom:
                odom_to_rospy_publish(self.odom_pub, odom_position, odom_yaw)
            self.pose_pub.publish(
                camera_pose(body_error, rospy.Time.now(), yaw_error)
            )
            if predicate():
                return
            rospy.sleep(0.025)
        self.fail('%s not met within %.1fs' % (description, timeout))

    def publish_aligned(
        self, duration, body_error=None, yaw_error=0.0,
        odom_position=(0.0, 0.0, 0.0), odom_yaw=0.0, publish_odom=True,
        state_mode='OFFBOARD', state_connected=True, state_armed=True,
        error=None,
    ):
        if error is not None:
            body_error = error
        if body_error is None:
            body_error = self.aligned_error
        deadline = time.time() + duration
        while time.time() < deadline and not rospy.is_shutdown():
            self.publish_state(state_connected, state_armed, state_mode)
            if publish_odom:
                odom_to_rospy_publish(self.odom_pub, odom_position, odom_yaw)
            self.pose_pub.publish(
                camera_pose(body_error, rospy.Time.now(), yaw_error)
            )
            rospy.sleep(0.025)

    def latest_preview(self):
        with self.lock:
            return self.previews[-1] if self.previews else None

    def latest_actual(self):
        with self.lock:
            return self.actual[-1] if self.actual else None

    def latest_command(self):
        with self.lock:
            return self.commands[-1] if self.commands else None

    def status_seen(self, marker, since_index=0):
        with self.lock:
            return any(
                marker in message.data
                for message in self.statuses[since_index:]
            )

    def status_index(self, marker):
        with self.lock:
            for index, message in enumerate(self.statuses):
                if marker in message.data:
                    return index
        return None

    def latest_position_preview(self):
        preview = self.latest_preview()
        if preview is None or preview.type_mask & PositionTarget.IGNORE_PX:
            return None
        return preview

    def test_position_hold_lifecycle(self):
        # ---- 1) 无定位时不切换 pos：视觉对齐但 odom 从未到达。
        no_odom_start = len(self.statuses)
        self.publish_aligned(1.2, publish_odom=False)
        rospy.sleep(0.2)
        # 定位从未到达 -> WAIT_LOCAL_POSITION，零速度输出，未锁定位置目标。
        self.wait_for(
            lambda: self.status_seen('WAIT_LOCAL_POSITION', no_odom_start),
            2.0, 'WAIT_LOCAL_POSITION before odom',
        )
        self.assertIsNone(self.latest_position_preview())

        # ---- 2) odom 到达并对齐后锁定位置目标。
        hold_start = len(self.statuses)
        self.publish_and_wait(
            lambda: self.status_seen('POS_HOLD', hold_start),
            6.0, 'POS_HOLD after aligned with odom',
        )
        preview = self.latest_position_preview()
        self.assertIsNotNone(preview)
        self.assertEqual(preview.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        # 屏蔽速度/加速度/yaw_rate。
        self.assertTrue(preview.type_mask & PositionTarget.IGNORE_VX)
        self.assertTrue(preview.type_mask & PositionTarget.IGNORE_VY)
        self.assertTrue(preview.type_mask & PositionTarget.IGNORE_VZ)
        self.assertTrue(preview.type_mask & PositionTarget.IGNORE_AFX)
        self.assertTrue(preview.type_mask & PositionTarget.IGNORE_AFY)
        self.assertTrue(preview.type_mask & PositionTarget.IGNORE_AFZ)
        self.assertTrue(preview.type_mask & PositionTarget.IGNORE_YAW_RATE)
        self.assertFalse(preview.type_mask & PositionTarget.IGNORE_PX)
        self.assertFalse(preview.type_mask & PositionTarget.IGNORE_YAW)
        # 机体在原点、误差为零、yaw=0：p_target = 0 + R*(0.8,0,0) - Rz(0)*(0.8,0,0) = 0。
        self.assertTrue(math.isclose(preview.position.x, 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(preview.position.y, 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(preview.position.z, 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(preview.yaw, 0.0, abs_tol=1e-6))
        # 位置模式下 cmd_vel 发布零值。
        command = self.latest_command()
        self.assertIsNotNone(command)
        self.assertEqual(command.twist.linear.x, 0.0)
        self.assertEqual(command.twist.angular.z, 0.0)
        # 实际输出与 preview 一致（位置型）。
        actual = self.latest_actual()
        self.assertIsNotNone(actual)
        self.assertFalse(actual.type_mask & PositionTarget.IGNORE_PX)
        self.assertTrue(math.isclose(actual.position.x, 0.0, abs_tol=1e-6))

        # ---- 3) 目标保持固定：odom 继续走，锁定值不漂移。
        drift_start = len(self.previews)
        self.publish_aligned(
            1.0, odom_position=(0.2, 0.1, 0.05), odom_yaw=0.3,
        )
        with self.lock:
            hold_samples = [
                message for message in self.previews[drift_start:]
                if not message.type_mask & PositionTarget.IGNORE_PX
            ]
        self.assertGreater(len(hold_samples), 5)
        for message in hold_samples:
            self.assertTrue(math.isclose(
                message.position.x, 0.0, abs_tol=1e-6
            ))
            self.assertTrue(math.isclose(
                message.position.y, 0.0, abs_tol=1e-6
            ))
            self.assertTrue(math.isclose(
                message.position.z, 0.0, abs_tol=1e-6
            ))
            self.assertTrue(math.isclose(message.yaw, 0.0, abs_tol=1e-6))

        # ---- 4) 视觉误差超过退出阈值并持续 -> 返回 ALIGN。
        exit_start = len(self.statuses)
        # 0.5m 前向误差 > 0.18m 退出阈值。
        self.publish_and_wait(
            lambda: self.status_seen('mode=ALIGN', exit_start),
            5.0, 'return to ALIGN after exit',
            body_error=(0.5, 0.0, 0.0),
        )
        # ALIGN 输出速度型 setpoint。
        velocity_previews = [
            message for message in self.previews
            if message.type_mask & PositionTarget.IGNORE_PX
        ]
        self.assertTrue(velocity_previews)

        # ---- 5) 近墙保护：marker x = 0.6 - 0.12 < 0.60 -> RETREAT。
        retreat_error = (self.marker_x_error(0.55), 0.0, 0.0)
        retreat_start = len(self.statuses)
        self.publish_and_wait(
            lambda: self.status_seen('RETREAT', retreat_start),
            4.0, 'RETREAT on near wall',
            body_error=retreat_error,
        )
        # 后退：速度输出 vx < 0，vy/vz/yaw_rate 为 0。
        # 后退起步受加速度限制（0.5 m/s^2），持续发布视觉保持 RETREAT 活动并等待速度建立。
        retreat_velocity_start = len(self.commands)
        self.publish_and_wait(
            lambda: any(
                message.twist.linear.x < -0.001
                for message in self.commands[retreat_velocity_start:]
            ),
            3.0, 'retreat velocity established',
            body_error=retreat_error,
        )
        rospy.sleep(0.2)
        with self.lock:
            retreat_command = self.commands[-1]
        self.assertLess(retreat_command.twist.linear.x, -0.001)
        self.assertEqual(retreat_command.twist.linear.y, 0.0)
        self.assertEqual(retreat_command.twist.linear.z, 0.0)
        self.assertEqual(retreat_command.twist.angular.z, 0.0)
        self.assertIsNone(self.latest_position_preview())

        # ---- 6) x 恢复至 >= 0.70 重新 ALIGN；0.65 仍在滞环内保持 RETREAT。
        hysteresis_error = (self.marker_x_error(0.65), 0.0, 0.0)
        self.publish_and_wait(
            lambda: self.status_seen('RETREAT', retreat_start),
            3.0, 'RETREAT hysteresis holds at 0.65m',
            body_error=hysteresis_error,
        )
        recover_error = (self.marker_x_error(0.75), 0.0, 0.0)
        recover_start = len(self.statuses)
        self.publish_and_wait(
            lambda: self.status_seen('mode=ALIGN', recover_start),
            4.0, 'back to ALIGN at 0.75m',
            body_error=recover_error,
        )

        # ---- 7) 硬门控：退出 OFFBOARD 清目标，输出零。
        gate_start = len(self.statuses)
        self.publish_and_wait(
            lambda: self.status_seen('WAIT_OFFBOARD', gate_start),
            4.0, 'WAIT_OFFBOARD under manual mode',
            state_mode='MANUAL',
        )
        gate_zero_seen = self._wait_nothrow(
            lambda: self.latest_actual().velocity.x == 0.0
            and self.latest_actual().type_mask & PositionTarget.IGNORE_PX,
            1.0,
        )
        self.assertTrue(gate_zero_seen)

    def _wait_nothrow(self, predicate, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                if predicate():
                    return True
            except AttributeError:
                pass
            rospy.sleep(0.01)
        return False

    def test_pose_loss_behavior(self):
        # 先进入 POS_HOLD（定位有效）。
        enter_start = len(self.statuses)
        self.publish_and_wait(
            lambda: self.status_seen('POS_HOLD', enter_start), 8.0,
            'POS_HOLD entered',
        )
        # 定位保持发布，视觉丢失 -> 仍保持固定目标（位置 setpoint 继续）。
        vision_loss_start = len(self.previews)
        deadline = time.time() + 0.6
        while time.time() < deadline and not rospy.is_shutdown():
            self.publish_state()
            odom_to_rospy_publish(self.odom_pub, (0.0, 0.0, 0.0), 0.0)
            rospy.sleep(0.025)
        with self.lock:
            loss_samples = [
                message for message in self.previews[vision_loss_start:]
                if not message.type_mask & PositionTarget.IGNORE_PX
            ]
        self.assertGreater(len(loss_samples), 5)
        for message in loss_samples:
            self.assertTrue(math.isclose(
                message.position.x, 0.0, abs_tol=1e-6
            ))
        self.assertTrue(self.status_seen('POS_HOLD'))

        # 定位也丢失 -> WAIT_LOCAL_POSITION，位置目标清除。
        both_loss_start = len(self.statuses)
        deadline = time.time() + 0.8
        while time.time() < deadline and not rospy.is_shutdown():
            self.publish_state()
            rospy.sleep(0.025)
        self.wait_for(
            lambda: self.status_seen('WAIT_LOCAL_POSITION', both_loss_start),
            2.0, 'WAIT_LOCAL_POSITION after losing odom',
        )
        rospy.sleep(0.2)
        self.assertIsNone(self.latest_position_preview())

    def test_retreat_pose_loss_brakes(self):
        # 直接触发近墙。
        retreat_error = (self.marker_x_error(0.55), 0.0, 0.0)
        retreat_start = len(self.statuses)
        self.publish_and_wait(
            lambda: self.status_seen('RETREAT', retreat_start), 4.0,
            'RETREAT entered',
            body_error=retreat_error,
        )
        # 后退中丢标：速度应按丢标减速收零。
        brake_start = len(self.commands)
        deadline = time.time() + 1.5
        while time.time() < deadline and not rospy.is_shutdown():
            self.publish_state()
            rospy.sleep(0.025)
        with self.lock:
            braking = [
                message for message in self.commands[brake_start:]
            ]
        # 至少出现过负速度，并且最终回到零。
        self.assertTrue(any(
            message.twist.linear.x < -0.001 for message in braking
        ))
        final_x = braking[-1].twist.linear.x
        self.assertTrue(
            math.isclose(final_x, 0.0, abs_tol=1e-9),
            'retreat pose-loss braking should reach zero, got %s' % final_x,
        )
        self.assertTrue(self.status_seen('RETREAT_WAIT_POSE')
                        or self.status_seen('BRAKING_POSE_LOSS'))

    def test_localization_rejection_clears_cache(self):
        # 正常进入 POS_HOLD。
        enter_start = len(self.statuses)
        self.publish_and_wait(
            lambda: self.status_seen('POS_HOLD', enter_start), 8.0,
            'POS_HOLD entered',
        )
        # 发送非法 odom（NaN 位置）：缓存被清，定位失效。
        reject_start = len(self.statuses)
        deadline = time.time() + 0.5
        while time.time() < deadline and not rospy.is_shutdown():
            self.publish_state()
            message = Odometry()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'odom'
            message.pose.pose.position.x = float('nan')
            self.odom_pub.publish(message)
            rospy.sleep(0.025)
        self.wait_for(
            lambda: self.status_seen('WAIT_LOCAL_POSITION', reject_start),
            2.0, 'WAIT_LOCAL_POSITION after NaN odom',
        )
        # 恢复前旧缓存不可复用：即使视觉继续有效也不切换 pos。
        self.publish_aligned(0.8, publish_odom=False)
        rospy.sleep(0.4)
        self.assertIsNone(self.latest_position_preview())
        # 恢复定位后可重新对齐并锁定。
        recover_start = len(self.statuses)
        self.publish_and_wait(
            lambda: self.status_seen('POS_HOLD', recover_start),
            8.0, 'POS_HOLD re-enters after localization recovers',
        )

    @staticmethod
    def marker_x_error(marker_x):
        # body_error.x = marker_x - target_x(0.8)
        return marker_x - 0.8


if __name__ == '__main__':
    rospy.init_node('position_hold_test')
    rostest.rosrun(
        'front_marker_control', 'position_hold', PositionHoldTest
    )
