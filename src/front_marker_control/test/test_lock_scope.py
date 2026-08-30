#!/usr/bin/env python3

import threading
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
import rospy

from front_marker_control.control_node import FrontMarkerControlNode


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
        node.lock = threading.RLock()
        node.last_accepted_source_stamp = None
        node.last_pid_source_stamp = None
        node.max_pose_latency_sec = 0.5
        node.future_stamp_tolerance_sec = 0.1
        node.tf_buffer = transform
        node.base_frame = 'base_link'
        node.tf_lookup_timeout_sec = 0.05
        node.target_xyz_body = (0.8, 0.0, 0.0)
        node.setpoint_publish_rate_hz = 20.0
        node.pid_dt_max_sec = 0.5
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
        node.position_error_pub = position_publisher or RecordingPublisher()
        node.yaw_error_pub = RecordingPublisher()
        return node

    def control_node(self, command_publisher=None, status_publisher=None):
        node = FrontMarkerControlNode.__new__(FrontMarkerControlNode)
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
        node.command_pub = command_publisher or RecordingPublisher()
        node.preview_pub = RecordingPublisher()
        node.setpoint_pub = None
        node.last_status = ''
        node.last_motion_state = None
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


if __name__ == '__main__':
    unittest.main()
