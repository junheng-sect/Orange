#!/usr/bin/env python3
"""Verify adaptive ROI coordinates, fallback timing, and disabled behavior."""

import re
import threading
import unittest

from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
import rostest
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


MARKER_CELLS = (
    '0000000',
    '0001000',
    '0101010',
    '0010100',
    '0101100',
    '0010000',
    '0000000',
)


def marker_image_at(x_start, y_start, cell_size=12):
    image = np.full((480, 640), 255, dtype=np.uint8)
    for row, bits in enumerate(MARKER_CELLS):
        for column, bit in enumerate(bits):
            value = 255 if bit == '1' else 0
            y0 = y_start + row * cell_size
            x0 = x_start + column * cell_size
            image[y0:y0 + cell_size, x0:x0 + cell_size] = value
    return image


class RoiTrackingTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.enabled_poses = {}
        self.disabled_poses = {}
        self.enabled_statuses = []
        self.disabled_statuses = []
        self.annotated_images = []
        self.image_pub = rospy.Publisher('/test_roi/image', Image, queue_size=1)
        self.info_pub = rospy.Publisher('/test_roi/camera_info', CameraInfo, queue_size=1)
        rospy.Subscriber(
            '/test_roi/enabled/pose', PoseStamped,
            lambda message: self.pose_callback(self.enabled_poses, message), queue_size=30,
        )
        rospy.Subscriber(
            '/test_roi/disabled/pose', PoseStamped,
            lambda message: self.pose_callback(self.disabled_poses, message), queue_size=30,
        )
        rospy.Subscriber(
            '/test_roi/enabled/status', String,
            lambda message: self.status_callback(self.enabled_statuses, message), queue_size=30,
        )
        rospy.Subscriber(
            '/test_roi/disabled/status', String,
            lambda message: self.status_callback(self.disabled_statuses, message), queue_size=30,
        )
        rospy.Subscriber(
            '/test_roi/enabled/annotated', Image,
            self.annotated_callback, queue_size=10,
        )

    def pose_callback(self, target, message):
        with self.lock:
            target[message.header.stamp.to_nsec()] = message

    def status_callback(self, target, message):
        with self.lock:
            target.append(message.data)

    def annotated_callback(self, message):
        with self.lock:
            self.annotated_images.append(message)

    @staticmethod
    def camera_info(stamp):
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = 'usb_camera_optical_frame'
        message.width = 640
        message.height = 480
        message.distortion_model = 'plumb_bob'
        message.D = [0.0] * 5
        message.K = [741.67267, 0.0, 290.60259,
                     0.0, 741.44273, 217.48395,
                     0.0, 0.0, 1.0]
        message.R = [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]
        message.P = [741.67267, 0.0, 290.60259, 0.0,
                     0.0, 741.44273, 217.48395, 0.0,
                     0.0, 0.0, 1.0, 0.0]
        return message

    @staticmethod
    def image_message(image, stamp):
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = 'usb_camera_optical_frame'
        message.height, message.width = image.shape
        message.encoding = 'mono8'
        message.is_bigendian = False
        message.step = message.width
        message.data = image.tobytes()
        return message

    def publish_frame(self, image):
        stamp = rospy.Time.now()
        self.info_pub.publish(self.camera_info(stamp))
        self.image_pub.publish(self.image_message(image, stamp))
        rospy.sleep(0.16)
        return stamp.to_nsec()

    @staticmethod
    def wait_until(predicate, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return True
            rospy.sleep(0.05)
        return False

    def matching_pose(self, stamp_ns):
        with self.lock:
            return self.enabled_poses.get(stamp_ns), self.disabled_poses.get(stamp_ns)

    def test_tracking_roi_and_full_frame_reacquisition(self):
        self.assertTrue(self.wait_until(
            lambda: self.image_pub.get_num_connections() >= 2
            and self.info_pub.get_num_connections() >= 2,
        ))

        left = marker_image_at(60, 190)
        left_stamps = [self.publish_frame(left) for _ in range(5)]
        self.assertTrue(self.wait_until(lambda: any(
            re.search(r'roi=\d+,\d+,\d+x\d+ miss=0', text)
            for text in self.enabled_statuses
        )))
        self.assertTrue(self.wait_until(lambda: any(
            'roi=FULL disabled miss=0' in text for text in self.disabled_statuses
        )))

        matched = None
        for stamp_ns in reversed(left_stamps):
            enabled, disabled = self.matching_pose(stamp_ns)
            if enabled is not None and disabled is not None:
                matched = enabled, disabled
                break
        self.assertIsNotNone(matched, 'ROI and full-frame nodes have no matching pose')
        enabled, disabled = matched
        self.assertAlmostEqual(enabled.pose.position.x, disabled.pose.position.x, places=9)
        self.assertAlmostEqual(enabled.pose.position.y, disabled.pose.position.y, places=9)
        self.assertAlmostEqual(enabled.pose.position.z, disabled.pose.position.z, places=9)

        with self.lock:
            self.assertTrue(self.annotated_images)
            annotated = self.annotated_images[-1]
        self.assertEqual(annotated.encoding, 'bgr8')
        self.assertEqual((annotated.width, annotated.height), (640, 480))
        self.assertEqual(annotated.step, 640 * 3)

        # Move within the active ROI and compare again to prove ROI corner
        # offsets are restored before pose estimation.
        nearby_stamp = self.publish_frame(marker_image_at(90, 190))
        self.assertTrue(self.wait_until(
            lambda: all(item is not None for item in self.matching_pose(nearby_stamp))
        ))
        enabled, disabled = self.matching_pose(nearby_stamp)
        self.assertAlmostEqual(enabled.pose.position.x, disabled.pose.position.x, places=9)
        self.assertAlmostEqual(enabled.pose.position.y, disabled.pose.position.y, places=9)
        self.assertAlmostEqual(enabled.pose.position.z, disabled.pose.position.z, places=9)

        # A 14px marker still passes the existing 12px contour gate. Publish it
        # twice at the same center: the first detection shrinks the next search
        # region to the 128px minimum, and the second proves both the adaptive
        # ROI rate and the full-frame base rate preserve the valid target.
        small = marker_image_at(125, 225, cell_size=2)
        small_stamps = [self.publish_frame(small) for _ in range(2)]
        self.assertTrue(self.wait_until(
            lambda: all(item is not None for item in self.matching_pose(small_stamps[-1]))
        ))
        enabled, disabled = self.matching_pose(small_stamps[-1])
        self.assertAlmostEqual(enabled.pose.position.x, disabled.pose.position.x, places=9)
        self.assertAlmostEqual(enabled.pose.position.y, disabled.pose.position.y, places=9)
        self.assertAlmostEqual(enabled.pose.position.z, disabled.pose.position.z, places=9)
        with self.lock:
            enabled_statuses = list(self.enabled_statuses)
        self.assertTrue(any(
            text.startswith('DETECTED: ID33')
            and re.search(r'roi=\d+,\d+,128x128 miss=0', text)
            for text in enabled_statuses
        ))

        # The far-right marker is outside the base, 1.5x and 2.25x ROIs.
        # After three misses, the fourth callback must search full frame.
        right = marker_image_at(540, 190)
        right_stamps = [self.publish_frame(right) for _ in range(4)]
        self.assertTrue(self.wait_until(
            lambda: self.matching_pose(right_stamps[3])[0] is not None
        ))
        with self.lock:
            enabled_pose_stamps = set(self.enabled_poses)
            disabled_pose_stamps = set(self.disabled_poses)
            enabled_statuses = list(self.enabled_statuses)
        self.assertTrue(all(stamp in disabled_pose_stamps for stamp in right_stamps))
        self.assertTrue(all(stamp not in enabled_pose_stamps for stamp in right_stamps[:3]))
        self.assertIn(right_stamps[3], enabled_pose_stamps)
        self.assertTrue(any('miss=1' in text for text in enabled_statuses))
        self.assertTrue(any('miss=2' in text for text in enabled_statuses))
        self.assertTrue(any('miss=3 next=FULL' in text for text in enabled_statuses))
        self.assertTrue(any(
            text.startswith('DETECTED: ID33') and 'roi=FULL miss=0' in text
            for text in enabled_statuses
        ))


if __name__ == '__main__':
    rospy.init_node('test_roi_tracking')
    rostest.rosrun('front_marker_vision', 'roi_tracking', RoiTrackingTest)
