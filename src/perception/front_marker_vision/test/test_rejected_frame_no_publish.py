#!/usr/bin/env python3
"""Verify safe uncalibrated 2D detection, color annotation, and pose rejection."""

import threading
import unittest

import numpy as np
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

from front_marker_vision.msg import HomogeneousTransform


def marker_image():
    # DICT_5X5_1000 ID33 including its one-cell black border.
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


def color_marker_image():
    image = np.repeat(marker_image()[:, :, None], 3, axis=2)
    # A colored strip outside the Marker makes grayscale-to-BGR regressions visible.
    image[400:480, :, :] = np.array([20, 80, 180], dtype=np.uint8)
    return image


class RejectedFrameNoPublishTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.pose_count = 0
        self.transform_count = 0
        self.statuses = []
        self.processing_times = []
        self.annotated_images = []
        self.marker_tf_count = 0
        self.image_pub = rospy.Publisher(
            '/test_front_marker/image', Image, queue_size=1
        )
        self.camera_info_pub = rospy.Publisher(
            '/test_front_marker/camera_info', CameraInfo, queue_size=1, latch=True
        )
        self.transform_sub = rospy.Subscriber(
            '/test_front_marker/transform_debug',
            HomogeneousTransform,
            self.transform_callback,
            queue_size=10,
        )
        self.pose_sub = rospy.Subscriber(
            '/test_front_marker/pose', PoseStamped, self.pose_callback, queue_size=10
        )
        self.status_sub = rospy.Subscriber(
            '/test_front_marker/status',
            String,
            self.status_callback,
            queue_size=10,
        )
        self.processing_sub = rospy.Subscriber(
            '/test_front_marker/processing_time_ms',
            Float64,
            self.processing_callback,
            queue_size=20,
        )
        self.annotated_sub = rospy.Subscriber(
            '/test_front_marker/annotated',
            Image,
            self.annotated_callback,
            queue_size=10,
        )
        self.tf_sub = rospy.Subscriber(
            '/tf', TFMessage, self.tf_callback, queue_size=20
        )

    def tf_callback(self, message):
        with self.lock:
            self.marker_tf_count += sum(
                transform.child_frame_id == 'rejected_front_marker'
                for transform in message.transforms
            )

    def transform_callback(self, _message):
        with self.lock:
            self.transform_count += 1

    def pose_callback(self, _message):
        with self.lock:
            self.pose_count += 1

    def status_callback(self, message):
        with self.lock:
            self.statuses.append(message.data)

    def processing_callback(self, message):
        with self.lock:
            self.processing_times.append(message.data)

    def annotated_callback(self, message):
        with self.lock:
            self.annotated_images.append(message)

    @staticmethod
    def camera_info(stamp, calibrated=True):
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = 'usb_camera_optical_frame'
        message.width = 640
        message.height = 480
        message.distortion_model = 'plumb_bob'
        message.D = [0.0] * 5
        if calibrated:
            message.K = [741.67267, 0.0, 290.60259,
                         0.0, 741.44273, 217.48395,
                         0.0, 0.0, 1.0]
        message.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.P = [741.67267, 0.0, 290.60259, 0.0,
                     0.0, 741.44273, 217.48395, 0.0,
                     0.0, 0.0, 1.0, 0.0]
        return message

    @staticmethod
    def image_message(image, stamp):
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = 'usb_camera_optical_frame'
        message.height, message.width = image.shape[:2]
        message.encoding = 'mono8' if image.ndim == 2 else 'bgr8'
        message.is_bigendian = False
        message.step = message.width if image.ndim == 2 else message.width * 3
        message.data = image.tobytes()
        return message

    def test_rejected_frame_does_not_publish(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.image_pub.get_num_connections() and self.camera_info_pub.get_num_connections():
                break
            rospy.sleep(0.05)
        self.assertGreater(self.image_pub.get_num_connections(), 0)
        self.assertGreater(self.camera_info_pub.get_num_connections(), 0)
        self.assertNotIn(
            '/test_front_marker/disabled_annotated',
            dict(rospy.get_published_topics()),
        )

        image = color_marker_image()

        # An uncalibrated CameraInfo must still allow ID33 2D detection and a
        # color annotated image, but it must never enable metric pose output.
        for _ in range(10):
            stamp = rospy.Time.now()
            self.camera_info_pub.publish(self.camera_info(stamp, calibrated=False))
            self.image_pub.publish(self.image_message(image, stamp))
            rospy.sleep(0.05)

        deadline = rospy.Time.now() + rospy.Duration(3.0)
        detected_2d = False
        annotated = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                detected_2d = any(
                    text.startswith('DETECTED_2D: ID33')
                    and 'POSE_DISABLED: INVALID_CAMERA_INFO' in text
                    for text in self.statuses
                )
                if self.annotated_images:
                    annotated = self.annotated_images[-1]
            if detected_2d and annotated is not None:
                break
            rospy.sleep(0.05)

        self.assertTrue(detected_2d, 'uncalibrated ID33 was not detected in 2D')
        self.assertIsNotNone(annotated, 'no annotated image was published')
        self.assertEqual(annotated.encoding, 'bgr8')
        annotated_array = np.frombuffer(annotated.data, dtype=np.uint8)
        annotated_array = annotated_array.reshape(annotated.height, annotated.step)
        annotated_array = annotated_array[:, :annotated.width * 3]
        annotated_array = annotated_array.reshape(annotated.height, annotated.width, 3)
        sample = annotated_array[430:470, 540:620].mean(axis=(0, 1))
        self.assertTrue(np.allclose(sample, [20.0, 80.0, 180.0], atol=2.0), sample)

        # A mono8 source must stay grayscale through detection and only be
        # expanded to BGR when the throttled annotated output is requested.
        with self.lock:
            self.annotated_images = []
        image = marker_image()
        for _ in range(10):
            stamp = rospy.Time.now()
            self.camera_info_pub.publish(self.camera_info(stamp, calibrated=False))
            self.image_pub.publish(self.image_message(image, stamp))
            rospy.sleep(0.05)
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        mono_annotated = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.annotated_images:
                    mono_annotated = self.annotated_images[-1]
            if mono_annotated is not None:
                break
            rospy.sleep(0.05)
        self.assertIsNotNone(mono_annotated, 'no mono8 annotated image was published')
        self.assertEqual(mono_annotated.encoding, 'bgr8')
        mono_array = np.frombuffer(mono_annotated.data, dtype=np.uint8)
        mono_array = mono_array.reshape(mono_annotated.height, mono_annotated.step)
        mono_array = mono_array[:, :mono_annotated.width * 3]
        mono_array = mono_array.reshape(mono_annotated.height, mono_annotated.width, 3)
        mono_sample = mono_array[430:470, 540:620].mean(axis=(0, 1))
        self.assertTrue(np.allclose(mono_sample, [255.0, 255.0, 255.0], atol=1.0),
                        mono_sample)
        with self.lock:
            self.assertEqual(self.pose_count, 0)
            self.assertEqual(self.transform_count, 0)
            self.assertEqual(self.marker_tf_count, 0)
            self.assertTrue(self.processing_times)
            self.assertTrue(all(np.isfinite(value) and value >= 0.0
                                for value in self.processing_times))
            self.statuses = []

        # Once valid intrinsics arrive, the deliberately impossible contour
        # threshold must still reject the pose and publish no transform.
        for _ in range(10):
            stamp = rospy.Time.now()
            self.camera_info_pub.publish(self.camera_info(stamp, calibrated=True))
            self.image_pub.publish(self.image_message(image, stamp))
            rospy.sleep(0.05)

        deadline = rospy.Time.now() + rospy.Duration(3.0)
        rejected = False
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                rejected = any(
                    text.startswith('REJECTED_CONTOUR')
                    for text in self.statuses
                )
            if rejected:
                break
            rospy.sleep(0.05)

        self.assertTrue(rejected, 'synthetic ID33 was not rejected by the contour gate')
        rospy.sleep(0.2)
        with self.lock:
            self.assertEqual(self.pose_count, 0)
            self.assertEqual(self.transform_count, 0)
            self.assertEqual(self.marker_tf_count, 0)
            rejected_statuses = [
                text for text in self.statuses
                if text.startswith('REJECTED_CONTOUR')
            ]
            self.assertTrue(rejected_statuses)
            self.assertTrue(
                all('branch_flips=0' in text for text in rejected_statuses),
                'IPPE status did not expose a zero confirmed flip count',
            )


if __name__ == '__main__':
    rospy.init_node('test_rejected_frame_no_publish')
    rostest.rosrun(
        'front_marker_vision',
        'rejected_frame_no_publish',
        RejectedFrameNoPublishTest,
    )
