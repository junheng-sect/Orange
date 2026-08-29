#!/usr/bin/env python3
"""Publish readable, offset labels for selected TF frame origins in RViz."""

import math

from geometry_msgs.msg import Point
import rospy
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


class TfFrameLabelsNode:
    """Render TF frame names as offset text with leader lines."""

    _DEFAULT_TEXT_HEIGHT_M = 0.03
    _DEFAULT_LINE_WIDTH_M = 0.002
    _DEFAULT_PUBLISH_RATE_HZ = 10.0
    _MARKER_LIFETIME_SEC = 0.5

    def __init__(self):
        self.fixed_frame = self._required_string_param('~fixed_frame', 'map')
        topic = self._required_string_param(
            '~topic', '/seeker_visualization/tf_frame_labels'
        )
        self.text_height_m = self._positive_float_param(
            '~text_height_m', self._DEFAULT_TEXT_HEIGHT_M
        )
        self.line_width_m = self._positive_float_param(
            '~line_width_m', self._DEFAULT_LINE_WIDTH_M
        )
        publish_rate_hz = self._positive_float_param(
            '~publish_rate_hz', self._DEFAULT_PUBLISH_RATE_HZ
        )

        self.labels = (
            self._label_spec(
                '~map_frame', 'map', '~map_offset', (-0.055, -0.055, 0.025),
                (1.0, 1.0, 1.0),
            ),
            self._label_spec(
                '~base_frame', 'base_link', '~base_offset', (0.055, -0.040, 0.035),
                (0.2, 0.8, 1.0),
            ),
            self._label_spec(
                '~camera_frame', 'usb_camera_optical_frame', '~camera_offset',
                (-0.055, 0.045, 0.060), (1.0, 0.4, 1.0),
            ),
            self._label_spec(
                '~marker_frame', 'front_marker', '~marker_offset',
                (0.045, 0.045, 0.035), (1.0, 0.85, 0.2),
            ),
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / publish_rate_hz), self.timer_callback
        )
        rospy.loginfo(
            'tf_frame_labels_node started | fixed=%s | topic=%s | text=%.3fm',
            self.fixed_frame,
            topic,
            self.text_height_m,
        )

    @staticmethod
    def _required_string_param(name, default):
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
    def _vector_param(name, default):
        values = rospy.get_param(name, list(default))
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise ValueError('{} must contain exactly three values'.format(name))
        vector = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in vector):
            raise ValueError('{} values must be finite'.format(name))
        return vector

    def _label_spec(self, frame_param, frame_default, offset_param, offset_default, color):
        return {
            'frame': self._required_string_param(frame_param, frame_default),
            'offset': self._vector_param(offset_param, offset_default),
            'color': color,
        }

    def _origin_in_fixed_frame(self, frame):
        if frame == self.fixed_frame:
            return Point(x=0.0, y=0.0, z=0.0)
        transform = self.tf_buffer.lookup_transform(
            self.fixed_frame, frame, rospy.Time(0), rospy.Duration(0.02)
        )
        translation = transform.transform.translation
        return Point(x=translation.x, y=translation.y, z=translation.z)

    @staticmethod
    def _offset_point(origin, offset):
        return Point(
            x=origin.x + offset[0],
            y=origin.y + offset[1],
            z=origin.z + offset[2],
        )

    @staticmethod
    def _set_color(marker, color, alpha):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = alpha

    def _common_marker(self, stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.fixed_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(self._MARKER_LIFETIME_SEC)
        return marker

    def _text_marker(self, stamp, marker_id, label, position):
        marker = self._common_marker(
            stamp, 'tf_frame_label_text', marker_id, Marker.TEXT_VIEW_FACING
        )
        marker.pose.position = position
        marker.scale.z = self.text_height_m
        marker.text = label['frame']
        self._set_color(marker, label['color'], 1.0)
        return marker

    def _leader_marker(self, stamp, marker_id, label, origin, position):
        marker = self._common_marker(
            stamp, 'tf_frame_label_leaders', marker_id, Marker.LINE_LIST
        )
        marker.scale.x = self.line_width_m
        marker.points = [origin, position]
        self._set_color(marker, label['color'], 0.75)
        return marker

    def timer_callback(self, _event):
        stamp = rospy.Time.now()
        markers = []
        for marker_id, label in enumerate(self.labels):
            try:
                origin = self._origin_in_fixed_frame(label['frame'])
            except (
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                tf2_ros.LookupException,
            ) as exception:
                rospy.logwarn_throttle(
                    5.0,
                    "TF label waiting for '%s' in '%s': %s",
                    label['frame'],
                    self.fixed_frame,
                    exception,
                )
                continue
            position = self._offset_point(origin, label['offset'])
            markers.append(self._leader_marker(stamp, marker_id, label, origin, position))
            markers.append(self._text_marker(stamp, marker_id, label, position))
        self.publisher.publish(MarkerArray(markers=markers))


def main():
    rospy.init_node('tf_frame_labels_node')
    try:
        TfFrameLabelsNode()
        rospy.spin()
    except (KeyError, TypeError, ValueError) as exception:
        rospy.logfatal('tf_frame_labels_node failed: %s', exception)
        raise


if __name__ == '__main__':
    main()
