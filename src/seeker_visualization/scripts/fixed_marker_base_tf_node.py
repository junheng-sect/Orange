#!/usr/bin/env python3
"""Infer a stable map-to-base TF from a fixed Marker observation."""

import math
import threading

from front_marker_control.control import wrap_angle
from front_marker_control.geometry import marker_heading_body, position_error_body
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import numpy as np
import rospy
from tf.transformations import (
    euler_matrix,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    quaternion_slerp,
)
import tf2_ros


class OutlierRejected(ValueError):
    """Raised when a single visual sample exceeds the configured motion gate."""


class FixedMarkerBaseTfNode:
    """Publish filtered 6DoF or controller-error position/yaw base estimates."""

    _QUATERNION_NORM_TOLERANCE = 1.0e-3
    _POSE_METHODS = ('filtered_6d', 'error_yaw')

    def __init__(self):
        self.pose_topic = self._string_param('~pose_topic', '/front_marker/pose')
        self.map_frame = self._string_param('~map_frame', 'map')
        self.base_frame = self._string_param('~base_frame', 'base_link')
        self.camera_frame = self._string_param(
            '~camera_frame', 'usb_camera_optical_frame'
        )
        self.marker_frame = self._string_param('~marker_frame', 'front_marker')
        self.pose_method = self._string_param('~pose_method', 'filtered_6d')
        if self.pose_method not in self._POSE_METHODS:
            raise ValueError(
                'pose_method must be one of {}'.format(', '.join(self._POSE_METHODS))
            )
        self.filter_tau_sec = self._positive_float_param('~filter_tau_sec', 0.25)
        self.filter_reset_gap_sec = self._positive_float_param(
            '~filter_reset_gap_sec', 0.5
        )
        self.max_translation_jump_m = self._positive_float_param(
            '~max_translation_jump_m', 0.15
        )
        self.max_rotation_jump_rad = self._positive_float_param(
            '~max_rotation_jump_rad', math.radians(15.0)
        )
        self.tf_lookup_timeout_sec = self._positive_float_param(
            '~tf_lookup_timeout_sec', 0.05
        )
        self.target_xyz_body = self._vector3_param(
            '~target_xyz_body', (0.8, 0.0, 0.0)
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
        self.last_raw_translation = None
        self.last_raw_quaternion = None
        self.filtered_translation = None
        self.filtered_quaternion = None

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
            'output=%s -> %s | method=%s | tau=%.3fs | reset_gap=%.3fs | '
            'jump_gate=(%.3fm, %.1fdeg) | no hold',
            self.pose_topic,
            self.map_frame,
            self.marker_frame,
            self.map_frame,
            self.base_frame,
            self.pose_method,
            self.filter_tau_sec,
            self.filter_reset_gap_sec,
            self.max_translation_jump_m,
            math.degrees(self.max_rotation_jump_rad),
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

    @staticmethod
    def _vector3_param(name, default):
        values = rospy.get_param(name, list(default))
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise ValueError('{} must contain exactly three values'.format(name))
        result = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in result):
            raise ValueError('{} values must be finite'.format(name))
        return result

    @classmethod
    def _normalize_quaternion(cls, orientation, description='pose'):
        quaternion = np.asarray(
            [orientation.x, orientation.y, orientation.z, orientation.w],
            dtype=float,
        )
        if not np.isfinite(quaternion).all():
            raise ValueError('{} quaternion contains non-finite values'.format(description))
        norm_squared = float(np.dot(quaternion, quaternion))
        if abs(norm_squared - 1.0) > cls._QUATERNION_NORM_TOLERANCE:
            raise ValueError(
                '{} quaternion is not unit length (norm_squared={:.6f})'.format(
                    description, norm_squared
                )
            )
        return quaternion / math.sqrt(norm_squared)

    @classmethod
    def _matrix_from_pose(cls, pose, description='pose'):
        translation = np.asarray(
            [pose.position.x, pose.position.y, pose.position.z], dtype=float
        )
        if not np.isfinite(translation).all():
            raise ValueError('{} position contains non-finite values'.format(description))
        quaternion = cls._normalize_quaternion(pose.orientation, description)
        matrix = quaternion_matrix(quaternion)
        matrix[:3, 3] = translation
        return matrix

    @classmethod
    def _matrix_from_transform(cls, transform, description='transform'):
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = transform.rotation
        return cls._matrix_from_pose(pose, description)

    @staticmethod
    def _matrix_components(matrix):
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError('computed map-to-base matrix is invalid')
        translation = np.asarray(matrix[:3, 3], dtype=float).copy()
        quaternion = np.asarray(quaternion_from_matrix(matrix), dtype=float)
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or norm <= 0.0:
            raise ValueError('computed map-to-base quaternion is invalid')
        quaternion /= norm
        if quaternion[3] < 0.0:
            quaternion *= -1.0
        return translation, quaternion

    @staticmethod
    def _components_to_transform(
        translation, quaternion, stamp, parent_frame, child_frame
    ):
        output = TransformStamped()
        output.header.stamp = stamp
        output.header.frame_id = parent_frame
        output.child_frame_id = child_frame
        output.transform.translation.x = float(translation[0])
        output.transform.translation.y = float(translation[1])
        output.transform.translation.z = float(translation[2])
        output.transform.rotation.x = float(quaternion[0])
        output.transform.rotation.y = float(quaternion[1])
        output.transform.rotation.z = float(quaternion[2])
        output.transform.rotation.w = float(quaternion[3])
        return output

    @staticmethod
    def _matrix_to_pose(matrix):
        translation, quaternion = FixedMarkerBaseTfNode._matrix_components(matrix)
        pose = Pose()
        pose.position.x = float(translation[0])
        pose.position.y = float(translation[1])
        pose.position.z = float(translation[2])
        pose.orientation.x = float(quaternion[0])
        pose.orientation.y = float(quaternion[1])
        pose.orientation.z = float(quaternion[2])
        pose.orientation.w = float(quaternion[3])
        return pose

    @staticmethod
    def _quaternion_angle(first, second):
        dot = abs(float(np.dot(first, second)))
        return 2.0 * math.acos(max(-1.0, min(1.0, dot)))

    def _filtered_6d_candidate(
        self, map_to_marker, base_to_camera, camera_to_marker
    ):
        map_to_base = np.matmul(
            np.matmul(map_to_marker, inverse_matrix(camera_to_marker)),
            inverse_matrix(base_to_camera),
        )
        return self._matrix_components(map_to_base)

    def _error_yaw_candidate(
        self, map_to_marker, base_to_camera, camera_to_marker
    ):
        base_to_marker = np.matmul(base_to_camera, camera_to_marker)
        pose_body = self._matrix_to_pose(base_to_marker)
        error = np.asarray(
            position_error_body(pose_body, self.target_xyz_body), dtype=float
        )
        observed_marker_body = error + np.asarray(self.target_xyz_body, dtype=float)
        marker_heading = marker_heading_body(pose_body.orientation)

        direction_map = -np.asarray(map_to_marker[:3, 2], dtype=float)
        if math.hypot(direction_map[0], direction_map[1]) <= 1.0e-12:
            raise ValueError('fixed Marker normal has no horizontal projection')
        fixed_heading_map = math.atan2(direction_map[1], direction_map[0])
        base_yaw = wrap_angle(fixed_heading_map - marker_heading)
        map_to_base = euler_matrix(0.0, 0.0, base_yaw)
        map_to_base[:3, 3] = (
            np.asarray(map_to_marker[:3, 3], dtype=float)
            - np.matmul(map_to_base[:3, :3], observed_marker_body)
        )
        return self._matrix_components(map_to_base)

    def _filter_candidate(self, translation, quaternion, stamp):
        if self.last_published_stamp is None:
            reset = True
        else:
            gap = (stamp - self.last_published_stamp).to_sec()
            reset = gap > self.filter_reset_gap_sec

        if reset:
            self.last_raw_translation = translation.copy()
            self.last_raw_quaternion = quaternion.copy()
            self.filtered_translation = translation.copy()
            self.filtered_quaternion = quaternion.copy()
            return self.filtered_translation.copy(), self.filtered_quaternion.copy()

        dt = (stamp - self.last_published_stamp).to_sec()
        translation_jump = float(
            np.linalg.norm(translation - self.last_raw_translation)
        )
        rotation_jump = self._quaternion_angle(
            quaternion, self.last_raw_quaternion
        )
        if translation_jump > self.max_translation_jump_m:
            raise OutlierRejected(
                'translation jump {:.3f}m exceeds {:.3f}m'.format(
                    translation_jump, self.max_translation_jump_m
                )
            )
        if rotation_jump > self.max_rotation_jump_rad:
            raise OutlierRejected(
                'rotation jump {:.1f}deg exceeds {:.1f}deg'.format(
                    math.degrees(rotation_jump),
                    math.degrees(self.max_rotation_jump_rad),
                )
            )

        alpha = 1.0 - math.exp(-dt / self.filter_tau_sec)
        self.filtered_translation = self.filtered_translation + alpha * (
            translation - self.filtered_translation
        )
        aligned_quaternion = quaternion.copy()
        if float(np.dot(self.filtered_quaternion, aligned_quaternion)) < 0.0:
            aligned_quaternion *= -1.0
        self.filtered_quaternion = np.asarray(
            quaternion_slerp(
                self.filtered_quaternion,
                aligned_quaternion,
                alpha,
                shortestpath=True,
            ),
            dtype=float,
        )
        self.filtered_quaternion /= np.linalg.norm(self.filtered_quaternion)
        self.last_raw_translation = translation.copy()
        self.last_raw_quaternion = quaternion.copy()
        return self.filtered_translation.copy(), self.filtered_quaternion.copy()

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
            camera_to_marker = self._matrix_from_pose(
                message.pose, 'camera-to-Marker Pose'
            )
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
            map_to_marker = self._matrix_from_transform(
                map_to_marker_message.transform, 'map-to-Marker TF'
            )
            base_to_camera = self._matrix_from_transform(
                base_to_camera_message.transform, 'base-to-camera TF'
            )
            if self.pose_method == 'filtered_6d':
                translation, quaternion = self._filtered_6d_candidate(
                    map_to_marker, base_to_camera, camera_to_marker
                )
            else:
                translation, quaternion = self._error_yaw_candidate(
                    map_to_marker, base_to_camera, camera_to_marker
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
            try:
                filtered_translation, filtered_quaternion = self._filter_candidate(
                    translation, quaternion, message.header.stamp
                )
            except OutlierRejected as exception:
                rospy.logwarn_throttle(
                    2.0, 'Rejecting fixed-marker outlier: %s', exception
                )
                return
            output = self._components_to_transform(
                filtered_translation,
                filtered_quaternion,
                message.header.stamp,
                self.map_frame,
                self.base_frame,
            )
            if rospy.is_shutdown():
                return
            try:
                self.broadcaster.sendTransform(output)
            except rospy.ROSException as exception:
                if not rospy.is_shutdown() and 'closed topic' not in str(exception):
                    raise
                return
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
