#!/usr/bin/env python3
"""Bench fake MAVROS for vz_step_control smoke tests (no FCU required).

Simulates the MAVROS proxies the step node consumes:
  /mavros/state                              @ 10 Hz
  /mavros/local_position/odom                @ 100 Hz (kinematic plant)
  /mavros/local_position/pose                @ 100 Hz
  /mavros/local_position/velocity_local      @ 100 Hz
  /mavros/setpoint_raw/target_local          @ 100 Hz
and answers /mavros/set_message_interval with success.

Scenario: POSCTL for --prestream seconds, then armed OFFBOARD until
/vz_step/status reports COMPLETE, then POSCTL again (bag closes on the
falling edge).
"""

import argparse
import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import MessageInterval, MessageIntervalResponse
from nav_msgs.msg import Odometry

from vz_step_control.msg import VzStepStatus


class FakeMavros:
    def __init__(self, prestream_sec):
        self.prestream_sec = prestream_sec
        self.start = rospy.Time.now().to_sec()
        self.x, self.y, self.z, self.yaw = 0.5, -0.25, 1.2, 0.3
        self.vz = 0.0
        self.offboard_latched = False
        self.leave_time = None
        self.complete_seen = False

        self.state_pub = rospy.Publisher(
            '/mavros/state', State, queue_size=10)
        self.odom_pub = rospy.Publisher(
            '/mavros/local_position/odom', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher(
            '/mavros/local_position/pose', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher(
            '/mavros/local_position/velocity_local', TwistStamped,
            queue_size=10)
        self.target_pub = rospy.Publisher(
            '/mavros/setpoint_raw/target_local', PositionTarget,
            queue_size=10)
        self.setpoint_sub = rospy.Subscriber(
            '/mavros/setpoint_raw/local', PositionTarget,
            self.setpoint_callback, queue_size=50)
        self.status_sub = rospy.Subscriber(
            '/vz_step/status', VzStepStatus, self.status_callback,
            queue_size=10)
        self.interval_srv = rospy.Service(
            '/mavros/set_message_interval', MessageInterval,
            self.interval_callback)

    def interval_callback(self, _request):
        response = MessageIntervalResponse()
        response.success = True
        return response

    def setpoint_callback(self, message):
        ignores_z = bool(message.type_mask & PositionTarget.IGNORE_PZ)
        if ignores_z:
            self.vz = message.velocity.z
        else:
            self.vz = 0.0
        self.x = message.position.x
        self.y = message.position.y
        self.yaw = message.yaw

    def status_callback(self, message):
        if message.state == VzStepStatus.COMPLETE:
            self.complete_seen = True

    def offboard_active(self):
        now = rospy.Time.now().to_sec()
        if now - self.start > self.prestream_sec:
            self.offboard_latched = True
        if self.offboard_latched and self.complete_seen and \
                self.leave_time is None:
            self.leave_time = now + 2.0
        if self.leave_time is not None and now > self.leave_time:
            return False
        return self.offboard_latched

    def publish_state(self, _event=None):
        message = State()
        message.header.stamp = rospy.Time.now()
        message.connected = True
        message.armed = self.offboard_active()
        message.mode = 'OFFBOARD' if self.offboard_active() else 'POSCTL'
        self.state_pub.publish(message)

    def publish_data(self, _event=None):
        dt = 0.01
        self.z += self.vz * dt
        stamp = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.twist.twist.linear.z = self.vz
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        velocity = TwistStamped()
        velocity.header.stamp = stamp
        velocity.twist.linear.z = self.vz
        self.velocity_pub.publish(velocity)

        target = PositionTarget()
        target.header.stamp = stamp
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.target_pub.publish(target)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prestream', type=float, default=4.0,
                        help='seconds of POSCTL before switching to OFFBOARD')
    args = parser.parse_args()
    rospy.init_node('fake_mavros')
    fake = FakeMavros(args.prestream)
    rospy.Timer(rospy.Duration(0.1), fake.publish_state)
    rospy.Timer(rospy.Duration(0.01), fake.publish_data)
    rospy.loginfo('fake_mavros ready; OFFBOARD after %.1f s', args.prestream)
    rospy.spin()


if __name__ == '__main__':
    main()
