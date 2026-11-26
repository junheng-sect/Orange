#!/usr/bin/env python3
"""Verify valid Marker Pose and TF outputs have identical transforms."""

import threading
import unittest

from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
import rostest
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


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


class MarkerTfPublicationTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.poses = {}
        self.transforms = {}
        self.image_pub = rospy.Publisher('/test_marker_tf/image', Image, queue_size=1)
        self.info_pub = rospy.Publisher(
            '/test_marker_tf/camera_info', CameraInfo, queue_size=1
        )
        self.pose_sub = rospy.Subscriber(
            '/test_marker_tf/pose', PoseStamped, self.pose_callback, queue_size=20
        )
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback, queue_size=50)

    def pose_callback(self, message):
        with self.lock:
            self.poses[message.header.stamp.to_nsec()] = message

    def tf_callback(self, message):
        with self.lock:
            for transform in message.transforms:
                if transform.child_frame_id == 'test_front_marker':
                    self.transforms[transform.header.stamp.to_nsec()] = transform

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

    def test_pose_and_tf_match(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.image_pub.get_num_connections() and self.info_pub.get_num_connections():
                break
            rospy.sleep(0.05)
        self.assertGreater(self.image_pub.get_num_connections(), 0)
        self.assertGreater(self.info_pub.get_num_connections(), 0)

        image = marker_image()
        for _ in range(20):
            stamp = rospy.Time.now()
            self.info_pub.publish(self.camera_info(stamp))
            self.image_pub.publish(self.image_message(image, stamp))
            rospy.sleep(0.05)

        matched = None
        deadline = rospy.Time.now() + rospy.Duration(4.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                common = set(self.poses).intersection(self.transforms)
                if common:
                    key = max(common)
                    matched = (self.poses[key], self.transforms[key])
            if matched is not None:
                break
            rospy.sleep(0.05)

        self.assertIsNotNone(matched, 'no matching Pose and marker TF were published')
        pose, transform = matched
        # TransformBroadcaster owns the transport sequence number, while the
        # source timestamp and parent frame must remain identical.
        self.assertEqual(transform.header.stamp, pose.header.stamp)
        self.assertEqual(transform.header.frame_id, pose.header.frame_id)
        self.assertEqual(transform.child_frame_id, 'test_front_marker')
        self.assertEqual(transform.transform.translation.x, pose.pose.position.x)
        self.assertEqual(transform.transform.translation.y, pose.pose.position.y)
        self.assertEqual(transform.transform.translation.z, pose.pose.position.z)
        self.assertEqual(transform.transform.rotation, pose.pose.orientation)


if __name__ == '__main__':
    rospy.init_node('test_marker_tf_publication')
    rostest.rosrun(
        'front_marker_vision',
        'marker_tf_publication',
        MarkerTfPublicationTest,
    )
