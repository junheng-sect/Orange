#!/usr/bin/env python3
"""Verify offset TF labels and their leader-line origins."""

import unittest

import rospy
import rostest
from visualization_msgs.msg import Marker, MarkerArray


class TfFrameLabelsTest(unittest.TestCase):
    def setUp(self):
        self.message = None
        self.subscriber = rospy.Subscriber(
            '/test_seeker_visualization/tf_frame_labels',
            MarkerArray,
            self.marker_callback,
            queue_size=1,
        )

    def marker_callback(self, message):
        if len(message.markers) == 8:
            self.message = message

    def wait_for_markers(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.message is not None:
                return self.message
            rospy.sleep(0.05)
        self.fail('Did not receive all TF label markers')

    def test_offsets_text_and_leaders(self):
        message = self.wait_for_markers()
        text_markers = {
            marker.text: marker
            for marker in message.markers
            if marker.type == Marker.TEXT_VIEW_FACING
        }
        leader_markers = {
            marker.id: marker
            for marker in message.markers
            if marker.type == Marker.LINE_LIST
        }

        self.assertEqual(
            set(text_markers), {'test_map', 'test_base', 'test_camera', 'test_marker'}
        )
        self.assertEqual(len(leader_markers), 4)

        expected_positions = {
            'test_map': (-0.1, 0.0, 0.02),
            'test_base': (0.1, 0.0, 0.13),
            'test_camera': (0.0, 0.1, 0.14),
            'test_marker': (1.0, 0.0, 0.15),
        }
        expected_origins = {
            0: (0.0, 0.0, 0.0),
            1: (0.0, 0.0, 0.1),
            2: (0.0, 0.0, 0.1),
            3: (1.0, 0.0, 0.1),
        }

        actual_positions = set()
        for label, expected in expected_positions.items():
            marker = text_markers[label]
            self.assertEqual(marker.header.frame_id, 'test_map')
            self.assertAlmostEqual(marker.scale.z, 0.03)
            actual = (
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            )
            actual_positions.add(actual)
            for actual_value, expected_value in zip(actual, expected):
                self.assertAlmostEqual(actual_value, expected_value)
        self.assertEqual(len(actual_positions), 4, 'label positions overlap')

        for marker_id, expected_origin in expected_origins.items():
            marker = leader_markers[marker_id]
            self.assertEqual(marker.header.frame_id, 'test_map')
            self.assertEqual(len(marker.points), 2)
            for actual_value, expected_value in zip(
                (
                    marker.points[0].x,
                    marker.points[0].y,
                    marker.points[0].z,
                ),
                expected_origin,
            ):
                self.assertAlmostEqual(actual_value, expected_value)


if __name__ == '__main__':
    rospy.init_node('test_tf_frame_labels')
    rostest.rosrun(
        'seeker_visualization', 'tf_frame_labels', TfFrameLabelsTest
    )
