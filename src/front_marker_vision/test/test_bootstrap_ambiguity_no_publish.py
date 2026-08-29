#!/usr/bin/env python3
"""Verify ambiguous bootstrap frames never escape before a reference-backed solve."""

import threading
import unittest

from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
import rostest
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

from front_marker_vision.msg import HomogeneousTransform


def marker_image():
    cells = (
        '0000000',
        '0001000',
        '0101010',
        '0010100',
        '0101100',
        '0010000',
        '0000000',
    )
    image = np.full((480, 640), 255, dtype=np.uint8)
    cell_size = 40
    x_start = 180
    y_start = 100
    for row, bits in enumerate(cells):
        for column, bit in enumerate(bits):
            value = 255 if bit == '1' else 0
            y0 = y_start + row * cell_size
            x0 = x_start + column * cell_size
            image[y0:y0 + cell_size, x0:x0 + cell_size] = value
    return image


class BootstrapAmbiguityNoPublishTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.pose_count = 0
        self.transform_count = 0
        self.marker_tf_count = 0
        self.processing_count = 0
        self.statuses = []
        self.image_pub = rospy.Publisher(
            '/test_bootstrap_ambiguity/image', Image, queue_size=1
        )
        self.info_pub = rospy.Publisher(
            '/test_bootstrap_ambiguity/camera_info', CameraInfo, queue_size=1, latch=True
        )
        self.pose_sub = rospy.Subscriber(
            '/test_bootstrap_ambiguity/pose', PoseStamped,
            self.pose_callback, queue_size=20
        )
        self.transform_sub = rospy.Subscriber(
            '/test_bootstrap_ambiguity/transform_debug', HomogeneousTransform,
            self.transform_callback, queue_size=20
        )
        self.processing_sub = rospy.Subscriber(
            '/test_bootstrap_ambiguity/processing_time', Float64,
            self.processing_callback, queue_size=20
        )
        self.status_sub = rospy.Subscriber(
            '/test_bootstrap_ambiguity/status', String,
            self.status_callback, queue_size=20
        )
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback, queue_size=50)

    def pose_callback(self, _message):
        with self.lock:
            self.pose_count += 1

    def transform_callback(self, _message):
        with self.lock:
            self.transform_count += 1

    def processing_callback(self, _message):
        with self.lock:
            self.processing_count += 1

    def status_callback(self, message):
        with self.lock:
            self.statuses.append(message.data)

    def tf_callback(self, message):
        with self.lock:
            self.marker_tf_count += sum(
                transform.child_frame_id == 'test_bootstrap_front_marker'
                for transform in message.transforms
            )

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

    def wait_for_connections(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.image_pub.get_num_connections() and self.info_pub.get_num_connections():
                return
            rospy.sleep(0.05)
        self.fail('vision node did not subscribe to the test inputs')

    def publish_and_wait(self, image):
        with self.lock:
            previous_processing_count = self.processing_count
        stamp = rospy.Time.now()
        self.image_pub.publish(self.image_message(image, stamp))
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.processing_count > previous_processing_count:
                    return
            rospy.sleep(0.01)
        self.fail('vision node did not process the published frame')

    def output_counts(self):
        with self.lock:
            return self.pose_count, self.transform_count, self.marker_tf_count

    def wait_for_output_after(self, baseline):
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            counts = self.output_counts()
            if all(current > previous for current, previous in zip(counts, baseline)):
                return counts
            rospy.sleep(0.01)
        self.fail('reference-backed frame did not publish Pose, debug transform, and marker TF')

    def assert_bootstrap_rejected(self, expected_streak):
        with self.lock:
            matching = [
                status for status in self.statuses
                if status.startswith('REJECTED_AMBIGUITY_PENDING: ID33')
                and 'ambiguous pose has no committed reference' in status
                and 'bootstrap confirmation %d/3' % expected_streak in status
                and 'ambiguous=1' in status
            ]
        self.assertTrue(matching, 'missing bootstrap rejection for streak %d' % expected_streak)

    def test_bootstrap_and_reacquisition_do_not_publish_ambiguous_pose(self):
        self.wait_for_connections()
        image = marker_image()
        info_stamp = rospy.Time.now()
        self.info_pub.publish(self.camera_info(info_stamp))
        rospy.sleep(0.1)

        initial_counts = self.output_counts()
        for streak in (1, 2, 3):
            self.publish_and_wait(image)
            rospy.sleep(0.05)
            self.assertEqual(self.output_counts(), initial_counts)
            self.assert_bootstrap_rejected(streak)

        self.publish_and_wait(image)
        first_published_counts = self.wait_for_output_after(initial_counts)

        rospy.sleep(0.30)
        with self.lock:
            self.statuses = []
        for streak in (1, 2, 3):
            self.publish_and_wait(image)
            rospy.sleep(0.05)
            self.assertEqual(self.output_counts(), first_published_counts)
            self.assert_bootstrap_rejected(streak)

        self.publish_and_wait(image)
        self.wait_for_output_after(first_published_counts)


if __name__ == '__main__':
    rospy.init_node('test_bootstrap_ambiguity_no_publish')
    rostest.rosrun(
        'front_marker_vision',
        'bootstrap_ambiguity_no_publish',
        BootstrapAmbiguityNoPublishTest,
    )
