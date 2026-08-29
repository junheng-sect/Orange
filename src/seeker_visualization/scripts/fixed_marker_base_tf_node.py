#!/usr/bin/env python3
"""Infer map-to-base TF from a fixed map marker and camera marker pose."""

import math
import threading

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rospy
from tf.transformations import inverse_matrix, quaternion_from_matrix, quaternion_matrix
import tf2_ros


class FixedMarkerBaseTfNode:
    """Solve T_map_base = T_map_marker * inv(T_camera_marker) * inv(T_base_camera)."""

    _QUATERNION_NORM_TOLERANCE = 1.0e-3

    def __init__(self):
        self.pose_topic = self._string_param('~pose_topic', '/front_marker/pose')
        self.map_frame = self._string_param('~map_frame', 'map')
        self.base_frame = self._string_param('~base_frame', 'base_link')
        self.camera_frame = self._string_param(
            '~camera_frame', 'usb_camera_optical_frame'
        )
        self.marker_frame = self._string_param('~marker_frame', 'front_marker')
        self.tf_lookup_timeout_sec = self._positive_float_param(
            '~tf_lookup_timeout_sec', 0.05
        )
        frames = {
            self.map_frame,
            self.base_frame,
            self.camera_frame,
            self.marker_frame,
        }
        if len(frames) != 4:
            raise ValueError('map, base, camera, and marker frames must all differ')

        self.lock = threading.Lock()
        self.last_published_stamp = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            self.pose_topic,
            PoseStamped,
            self.pose_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            'fixed_marker_base_tf_node started | pose=%s | fixed=%s -> %s | '
            'output=%s -> %s | raw 6DoF, no hold',
            self.pose_topic,
            self.map_frame,
            self.marker_frame,
            self.map_frame,
            self.base_frame,
        )

    @staticmethod
    def _string_param(name, default):
        value = str(rospy.get_param(name, default)).strip()
        if not value:
            raise ValueError('{} must not be empty'.format(name))
        return value

    @staticmethod
    def _positive_float_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('{} must be finite and greater than zero'.format(name))
        return value

    @classmethod
    def _pose_matrix(cls, pose):
        position = pose.position
        orientation = pose.orientation
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
            raise ValueError('pose contains non-finite values')
        norm_squared = (
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if abs(norm_squared - 1.0) > cls._QUATERNION_NORM_TOLERANCE:
            raise ValueError(
                'pose quaternion is not unit length (norm_squared={:.6f})'.format(
                    norm_squared
                )
            )
        norm = math.sqrt(norm_squared)
        quaternion = np.array(
            [
                orientation.x / norm,
                orientation.y / norm,
                orientation.z / norm,
                orientation.w / norm,
            ],
            dtype=float,
        )
        matrix = quaternion_matrix(quaternion)
        matrix[0, 3] = position.x
        matrix[1, 3] = position.y
        matrix[2, 3] = position.z
        return matrix

    @classmethod
    def _transform_matrix(cls, transform):
        class PoseView:
            pass

        pose = PoseView()
        pose.position = transform.translation
        pose.orientation = transform.rotation
        return cls._pose_matrix(pose)

    @staticmethod
    def _matrix_to_transform(matrix, stamp, parent_frame, child_frame):
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError('computed map-to-base matrix is invalid')
        quaternion = np.asarray(quaternion_from_matrix(matrix), dtype=float)
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or norm <= 0.0:
            raise ValueError('computed map-to-base quaternion is invalid')
        quaternion /= norm
        if quaternion[3] < 0.0:
            quaternion *= -1.0

        output = TransformStamped()
        output.header.stamp = stamp
        output.header.frame_id = parent_frame
        output.child_frame_id = child_frame
        output.transform.translation.x = float(matrix[0, 3])
        output.transform.translation.y = float(matrix[1, 3])
        output.transform.translation.z = float(matrix[2, 3])
        output.transform.rotation.x = float(quaternion[0])
        output.transform.rotation.y = float(quaternion[1])
        output.transform.rotation.z = float(quaternion[2])
        output.transform.rotation.w = float(quaternion[3])
        return output

    def _stamp_issue(self, message):
        if message.header.stamp == rospy.Time():
            return 'zero timestamp'
        if message.header.frame_id != self.camera_frame:
            return "parent frame '{}' does not match '{}'".format(
                message.header.frame_id, self.camera_frame
            )
        with self.lock:
            if (
                self.last_published_stamp is not None
                and message.header.stamp <= self.last_published_stamp
            ):
                return 'timestamp is duplicate or moving backwards'
        return None

    def pose_callback(self, message):
        issue = self._stamp_issue(message)
        if issue is not None:
            rospy.logwarn_throttle(2.0, 'Ignoring fixed-marker Pose: %s', issue)
            return

        try:
            camera_to_marker = self._pose_matrix(message.pose)
            timeout = rospy.Duration(self.tf_lookup_timeout_sec)
            map_to_marker_message = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.marker_frame,
                message.header.stamp,
                timeout,
            )
            base_to_camera_message = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                message.header.stamp,
                timeout,
            )
            map_to_marker = self._transform_matrix(
                map_to_marker_message.transform
            )
            base_to_camera = self._transform_matrix(
                base_to_camera_message.transform
            )
            map_to_base = np.matmul(
                np.matmul(map_to_marker, inverse_matrix(camera_to_marker)),
                inverse_matrix(base_to_camera),
            )
            output = self._matrix_to_transform(
                map_to_base,
                message.header.stamp,
                self.map_frame,
                self.base_frame,
            )
        except (
            ValueError,
            np.linalg.LinAlgError,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.LookupException,
        ) as exception:
            rospy.logwarn_throttle(
                2.0, 'Ignoring fixed-marker Pose: %s', exception
            )
            return

        with self.lock:
            if (
                self.last_published_stamp is not None
                and message.header.stamp <= self.last_published_stamp
            ):
                return
            self.broadcaster.sendTransform(output)
            self.last_published_stamp = message.header.stamp


def main():
    rospy.init_node('fixed_marker_base_tf_node')
    try:
        FixedMarkerBaseTfNode()
        rospy.spin()
    except (KeyError, TypeError, ValueError) as exception:
        rospy.logfatal('fixed_marker_base_tf_node failed: %s', exception)
        raise


if __name__ == '__main__':
    main()
