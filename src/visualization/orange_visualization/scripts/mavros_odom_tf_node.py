#!/usr/bin/env python3
"""Broadcast a validated px4_map-to-base transform from MAVROS Odometry."""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rospy
import tf2_ros


class MavrosOdometryTfNode:
    """Convert one authoritative Odometry stream into a TF edge."""

    _QUATERNION_NORM_TOLERANCE = 1.0e-3

    def __init__(self):
        self.odom_topic = str(
            rospy.get_param('~odom_topic', '/mavros/local_position/odom')
        )
        self.map_frame = str(rospy.get_param('~map_frame', 'px4_map'))
        self.base_frame = str(rospy.get_param('~base_frame', 'drone_base'))
        if not self.odom_topic:
            raise ValueError('odom_topic must not be empty')
        if not self.map_frame or not self.base_frame:
            raise ValueError('map_frame and base_frame must not be empty')
        if self.map_frame == self.base_frame:
            raise ValueError('map_frame and base_frame must differ')

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odometry_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            'mavros_odom_tf_node started | odom=%s | transform=%s -> %s',
            self.odom_topic,
            self.map_frame,
            self.base_frame,
        )

    def odometry_callback(self, message):
        if message.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring Odometry parent frame '%s'; expected '%s'",
                message.header.frame_id,
                self.map_frame,
            )
            return
        if message.child_frame_id != self.base_frame:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring Odometry child frame '%s'; expected '%s'",
                message.child_frame_id,
                self.base_frame,
            )
            return
        if message.header.stamp == rospy.Time():
            rospy.logwarn_throttle(2.0, 'Ignoring Odometry with a zero timestamp')
            return

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(2.0, 'Ignoring Odometry with non-finite pose values')
            return

        norm_squared = (
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if abs(norm_squared - 1.0) > self._QUATERNION_NORM_TOLERANCE:
            rospy.logwarn_throttle(
                2.0,
                'Ignoring Odometry with non-unit quaternion (norm_squared=%.6f)',
                norm_squared,
            )
            return

        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = orientation
        self.broadcaster.sendTransform(transform)


def main():
    rospy.init_node('mavros_odom_tf_node')
    try:
        MavrosOdometryTfNode()
        rospy.spin()
    except (KeyError, TypeError, ValueError) as exception:
        rospy.logfatal('mavros_odom_tf_node failed: %s', exception)
        raise


if __name__ == '__main__':
    main()
