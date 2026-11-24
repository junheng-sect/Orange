#!/usr/bin/env python3

import math
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
import rospy
import rostest
from std_msgs.msg import String


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
        self.statuses = []
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
        rospy.Subscriber(
            '/test_front_marker_control/status',
            String,
            self._append,
            callback_args=self.statuses,
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

    def publish_state(self, connected, armed, mode):
        message = State()
        message.connected = connected
        message.armed = armed
        message.mode = mode
        self.state_pub.publish(message)

    def publish_tracking_for(
        self,
        body_error,
        duration,
        connected=True,
        armed=True,
        mode='OFFBOARD',
    ):
        deadline = time.time() + duration
        last_stamp = None
        while time.time() < deadline and not rospy.is_shutdown():
            self.publish_state(connected, armed, mode)
            last_stamp = rospy.Time.now()
            self.pose_pub.publish(camera_pose(body_error, last_stamp))
            rospy.sleep(0.025)
        return last_stamp

    def latest_command(self):
        with self.lock:
            if not self.commands:
                return None
            message = self.commands[-1]
            return (
                message.twist.linear.x,
                message.twist.linear.y,
                message.twist.linear.z,
            )

    def latest_command_x(self):
        command = self.latest_command()
        return None if command is None else command[0]

    def latest_actual_x(self):
        with self.lock:
            return None if not self.actual else self.actual[-1].velocity.x

    def latest_error_x(self):
        with self.lock:
            return None if not self.errors else self.errors[-1].vector.x

    def command_count(self):
        with self.lock:
            return len(self.commands)

    def error_count(self):
        with self.lock:
            return len(self.errors)

    def command_slice(self, start):
        with self.lock:
            return list(self.commands[start:])

    def status_seen(self, marker):
        with self.lock:
            return any(marker in message.data for message in self.statuses)

    def assert_slew_bound(self, messages, rate, tolerance=0.008):
        self.assertGreaterEqual(len(messages), 2)
        for previous, current in zip(messages, messages[1:]):
            dt = (current.header.stamp - previous.header.stamp).to_sec()
            if dt <= 0.0:
                continue
            delta = math.sqrt(
                (current.twist.linear.x - previous.twist.linear.x) ** 2
                + (current.twist.linear.y - previous.twist.linear.y) ** 2
                + (current.twist.linear.z - previous.twist.linear.z) ** 2
            )
            self.assertLessEqual(delta, rate * min(dt, 0.1) + tolerance)

    def assert_actual_matches_commands(self, messages):
        with self.lock:
            actual_by_stamp = {
                item.header.stamp.to_nsec(): item.velocity.x for item in self.actual
            }
        matched = 0
        for command in messages:
            actual_x = actual_by_stamp.get(command.header.stamp.to_nsec())
            if actual_x is None:
                continue
            self.assertTrue(
                math.isclose(command.twist.linear.x, actual_x, abs_tol=1e-9)
            )
            matched += 1
        self.assertGreaterEqual(matched, 3)

    def test_slew_loss_braking_recovery_and_hard_gates(self):
        self.assertTrue(math.isclose(
            rospy.get_param('/sample_front_marker_control/pose_timeout_sec'),
            0.1,
        ))
        self.assertTrue(math.isclose(
            rospy.get_param('/sample_front_marker_control/max_pose_latency_sec'),
            0.5,
        ))
        self.assertTrue(math.isclose(
            rospy.get_param('/sample_front_marker_control/filter_dt_max_sec'),
            0.5,
        ))

        # A 250ms acquisition-to-arrival delay is accepted by the independent
        # 0.5s latency gate. A following sample with a 200ms source-stamp gap
        # keeps yaw-filter history because filter_dt_max_sec is independent
        # from the 0.1s receive-freshness timeout. With pure proportional
        # mapping, the raw x target is 0.5*0.1 = 0.05m/s.
        rospy.sleep(0.1)
        self.publish_state(True, True, 'OFFBOARD')
        first_delayed_stamp = rospy.Time.now() - rospy.Duration(0.25)
        self.pose_pub.publish(camera_pose((0.2, 0.0, 0.0), first_delayed_stamp))
        self.wait_for(
            lambda: self.latest_error_x() is not None
            and math.isclose(self.latest_error_x(), 0.2, abs_tol=1e-6),
            0.3,
        )
        state_deadline = time.time() + 0.08
        while time.time() < state_deadline:
            self.publish_state(True, True, 'OFFBOARD')
            rospy.sleep(0.01)
        self.wait_for(lambda: self.latest_command_x() > 0.0, 0.15)
        self.publish_state(True, True, 'OFFBOARD')
        second_delayed_stamp = first_delayed_stamp + rospy.Duration(0.2)
        self.pose_pub.publish(camera_pose((0.1, 0.0, 0.0), second_delayed_stamp))
        self.wait_for(
            lambda: math.isclose(self.latest_error_x(), 0.1, abs_tol=1e-6),
            0.3,
        )
        # Keep the same target fresh while the output slew limiter settles.
        # Waiting on a single sample for longer than pose_timeout_sec would
        # exercise pose-loss braking instead of proportional tracking.
        self.publish_tracking_for((0.1, 0.0, 0.0), 0.15)
        self.assertTrue(math.isclose(self.latest_command_x(), 0.05, abs_tol=0.004))

        # Hard MAVROS gates keep all final outputs at zero.
        hard_gate_stages = [
            (False, False, ''),
            (True, False, ''),
            (True, True, 'MANUAL'),
        ]
        for connected, armed, mode in hard_gate_stages:
            self.publish_tracking_for(
                (0.5, 0.0, 0.0),
                0.15,
                connected=connected,
                armed=armed,
                mode=mode,
            )
            self.assertTrue(math.isclose(self.latest_command_x(), 0.0, abs_tol=1e-9))
            self.assertTrue(math.isclose(self.latest_actual_x(), 0.0, abs_tol=1e-9))

        # A fresh Pose ramps from 0 to the 0.2m/s x limit at 0.5m/s^2.
        tracking_start = self.command_count()
        self.publish_tracking_for((0.5, 0.0, 0.0), 0.55)
        self.wait_for(
            lambda: math.isclose(self.latest_command_x(), 0.2, abs_tol=0.003),
            0.3,
        )
        tracking_messages = self.command_slice(tracking_start)
        self.assert_slew_bound(tracking_messages, 0.5)
        positive = [
            message for message in tracking_messages if message.twist.linear.x > 0.005
        ]
        settled = [
            message for message in tracking_messages if message.twist.linear.x >= 0.197
        ]
        self.assertTrue(positive)
        self.assertTrue(settled)
        rise_time = (settled[0].header.stamp - positive[0].header.stamp).to_sec()
        self.assertGreaterEqual(rise_time, 0.30)
        self.assertLessEqual(rise_time, 0.50)
        self.assert_actual_matches_commands(tracking_messages)
        self.wait_for(lambda: self.status_seen('TRACKING'), 1.0)

        # An invalid TF is a soft visual failure: it starts bounded braking,
        # rather than bypassing the limiter and jumping directly to zero.
        before_invalid = max(0, self.command_count() - 1)
        self.publish_state(True, True, 'OFFBOARD')
        self.pose_pub.publish(
            camera_pose((0.5, 0.0, 0.0), rospy.Time.now(), frame='missing_frame')
        )
        self.wait_for(lambda: self.latest_command_x() < 0.19, 0.3)
        invalid_messages = self.command_slice(before_invalid)
        self.assertGreater(invalid_messages[0].twist.linear.x, 0.0)
        self.assert_slew_bound(invalid_messages, 1.0)

        # Recovery uses the normal 0.5m/s^2 limit from the current brake value.
        error_start = self.error_count()
        self.publish_state(True, True, 'OFFBOARD')
        self.pose_pub.publish(camera_pose((0.5, 0.0, 0.0), rospy.Time.now()))
        self.wait_for(lambda: self.error_count() > error_start, 0.3)
        recovery_start = max(0, self.command_count() - 1)
        self.publish_tracking_for((0.5, 0.0, 0.0), 0.25)
        recovery_messages = self.command_slice(recovery_start)
        self.assert_slew_bound(recovery_messages, 0.5)

        # Natural Pose timeout starts near 0.1s, then 0.2m/s reaches zero in
        # about 0.2s using the 1.0m/s^2 loss-deceleration limit.
        self.publish_tracking_for((0.5, 0.0, 0.0), 0.25)
        self.wait_for(
            lambda: math.isclose(self.latest_command_x(), 0.2, abs_tol=0.003),
            0.3,
        )
        loss_start = rospy.Time.now()
        braking_start = self.command_count()
        self.wait_for(
            lambda: math.isclose(self.latest_command_x(), 0.0, abs_tol=1e-9),
            0.8,
        )
        braking_messages = self.command_slice(braking_start)
        self.assert_slew_bound(braking_messages, 1.0)
        decreasing = [
            message for message in braking_messages if message.twist.linear.x < 0.195
        ]
        self.assertTrue(decreasing)
        loss_delay = (decreasing[0].header.stamp - loss_start).to_sec()
        self.assertGreaterEqual(loss_delay, 0.04)
        self.assertLessEqual(loss_delay, 0.20)
        zero_message = next(
            message
            for message in braking_messages
            if math.isclose(message.twist.linear.x, 0.0, abs_tol=1e-9)
        )
        brake_time = (zero_message.header.stamp - decreasing[0].header.stamp).to_sec()
        self.assertGreaterEqual(brake_time, 0.10)
        self.assertLessEqual(brake_time, 0.30)
        self.assert_actual_matches_commands(braking_messages)
        self.wait_for(lambda: self.status_seen('BRAKING_POSE_LOSS'), 1.0)
        self.wait_for(lambda: self.status_seen('WAIT_POSE'), 1.0)

        # A hard gate still overrides the slew limiter within one control tick.
        self.publish_tracking_for((-0.5, 0.0, 0.0), 0.55)
        self.wait_for(
            lambda: math.isclose(self.latest_command_x(), -0.2, abs_tol=0.003),
            0.3,
        )
        hard_stop_time = rospy.Time.now()
        self.publish_state(True, True, 'MANUAL')
        self.wait_for(
            lambda: math.isclose(self.latest_command_x(), 0.0, abs_tol=1e-9)
            and math.isclose(self.latest_actual_x(), 0.0, abs_tol=1e-9),
            0.2,
        )
        with self.lock:
            hard_zero = next(
                message
                for message in self.commands
                if message.header.stamp >= hard_stop_time
                and math.isclose(message.twist.linear.x, 0.0, abs_tol=1e-9)
            )
        self.assertLessEqual((hard_zero.header.stamp - hard_stop_time).to_sec(), 0.12)
        self.wait_for(lambda: self.status_seen('WAIT_OFFBOARD'), 1.0)

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
