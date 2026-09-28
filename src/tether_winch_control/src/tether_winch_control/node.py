"""ROS adapter. Wall-clock watchdog keeps running when /clock stops."""
import math
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from tether_winch_control.msg import WinchStatus

from .controller import Config, Controller, finite_number
from .transport import make_transport


class WinchNode:
    def __init__(self):
        params = dict(rospy.get_param('~controller', {}))
        # ROS parameter server cannot represent None; YAML uses the string unset.
        for key in ('low_duty', 'high_duty'):
            if params.get(key) == 'unset':
                params[key] = None
        self.controller = Controller(Config(**params))
        self.transport = make_transport(rospy.get_param('~transport', 'mock'))
        rate = finite_number(rospy.get_param('~status_rate_hz', 20.0), 'status_rate_hz')
        if not 1 <= rate <= 100:
            raise ValueError('status_rate_hz must be between 1 and 100')
        self.period = 1.0 / rate
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.last_mode = None
        self.publisher = rospy.Publisher('~status', WinchStatus, queue_size=1)
        self.subscriber = rospy.Subscriber(
            rospy.get_param('~odom_topic', '/mavros/local_position/odom'),
            Odometry, self.on_odometry, queue_size=1, tcp_nodelay=True)
        self.service = rospy.Service('~set_enabled', SetBool, self.on_enabled)
        self.thread = threading.Thread(target=self.watchdog, name='winch-watchdog', daemon=True)
        rospy.on_shutdown(self.shutdown)
        self.thread.start()
        rospy.logwarn('Winch MOCK ONLY: no ESP32 connection or applied PWM feedback; '
                      'automatic control starts disabled.')

    def on_odometry(self, message):
        with self.lock:
            if self.stop_event.is_set():
                return
            p, q = message.pose.pose.position, message.pose.pose.orientation
            now = time.monotonic()
            status = self.controller.observe(
                message.header.stamp.to_sec(), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                message.header.frame_id, message.child_frame_id, now, rospy.get_time())
            self.transport.update(status, now)

    def on_enabled(self, request):
        with self.lock:
            if self.stop_event.is_set():
                return SetBoolResponse(success=False, message='node is shutting down')
            now, source_now = time.monotonic(), rospy.get_time()
            if request.data:
                success, reason = self.controller.enable(now, source_now)
            else:
                self.controller.disable()
                success, reason = True, 'disabled; no hardware stop is implemented in mock mode'
            self.transport.update(self.controller.status(), now)
            self.publish_status()
            return SetBoolResponse(success=success, message=reason)

    def publish_status(self):
        state = self.controller.status()
        message = WinchStatus()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.controller.config.frame_id
        message.mode, message.enabled = state.mode, state.enabled
        message.input_valid, message.reason = state.input_valid, state.reason
        message.distance_m = state.distance_m if state.distance_m is not None else math.nan
        message.radial_velocity_mps = (state.radial_velocity_mps
                                       if state.radial_velocity_mps is not None else math.nan)
        message.command_valid = state.target_duty is not None
        message.target_duty = state.target_duty if message.command_valid else math.nan
        message.transport = self.transport.name
        message.hardware_feedback_valid = False
        message.applied_duty = math.nan
        self.publisher.publish(message)
        if state.mode != self.last_mode:
            rospy.loginfo('Winch %s: %s (target duty: %s)', state.mode, state.reason,
                          state.target_duty)
            self.last_mode = state.mode

    def watchdog(self):
        while not self.stop_event.wait(self.period):
            if rospy.is_shutdown():
                return
            with self.lock:
                now = time.monotonic()
                state = self.controller.tick(now, rospy.get_time())
                self.transport.update(state, now)
                self.publish_status()

    def shutdown(self):
        self.stop_event.set()
        with self.lock:
            self.controller.disable()
            self.transport.update(self.controller.status(), time.monotonic())
        if threading.current_thread() is not self.thread:
            self.thread.join(timeout=1.0)


def main():
    rospy.init_node('tether_winch_control')
    try:
        node = WinchNode()
    except (ValueError, TypeError) as error:
        rospy.logfatal('Invalid winch configuration: %s', error)
        raise SystemExit(2)
    rospy.spin()
    node.shutdown()
