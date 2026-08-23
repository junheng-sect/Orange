#!/usr/bin/env python3
"""Manual saved-image check for equivalent Pose and debug-matrix output."""

import argparse
import math
import threading

import cv2
from cv_bridge import CvBridge
from front_marker_vision.msg import HomogeneousTransform
from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64


def camera_info(header):
    message = CameraInfo()
    message.header = header
    message.height = 480
    message.width = 640
    message.distortion_model = 'plumb_bob'
    message.D = [0.067792, 0.004847, 0.001479, -0.000168, 0.0]
    message.K = [741.67267, 0.0, 290.60259,
                 0.0, 741.44273, 217.48395,
                 0.0, 0.0, 1.0]
    message.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.P = [741.67267, 0.0, 290.60259, 0.0,
                 0.0, 741.44273, 217.48395, 0.0,
                 0.0, 0.0, 1.0, 0.0]
    return message


def quaternion_matrix(message):
    x = message.x
    y = message.y
    z = message.z
    w = message.w
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--rate', type=float, default=20.0)
    args = parser.parse_args()

    image = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
    if image is None or image.shape != (480, 640):
        raise RuntimeError('expected a readable 640x480 grayscale image')

    rospy.init_node('front_marker_vision_saved_image_check')
    bridge = CvBridge()
    image_pub = rospy.Publisher('/test_saved/image_raw', Image, queue_size=2)
    info_pub = rospy.Publisher('/test_saved/camera_info', CameraInfo, queue_size=2)
    lock = threading.Lock()
    poses = []
    transforms = []
    processing_times = []

    def append(target):
        def callback(message):
            with lock:
                target.append(message)
        return callback

    rospy.Subscriber('/front_marker/pose', PoseStamped, append(poses), queue_size=100)
    rospy.Subscriber(
        '/front_marker_vision/debug/homogeneous_transform',
        HomogeneousTransform,
        append(transforms),
        queue_size=100,
    )
    rospy.Subscriber(
        '/front_marker_vision/debug/processing_time_ms',
        Float64,
        append(processing_times),
        queue_size=100,
    )

    deadline = rospy.get_time() + 5.0
    while image_pub.get_num_connections() == 0 and rospy.get_time() < deadline:
        rospy.sleep(0.02)
    if image_pub.get_num_connections() == 0:
        raise RuntimeError('vision node did not subscribe to replay image topic')

    rate = rospy.Rate(args.rate)
    for sequence in range(args.frames):
        stamp = rospy.Time.now()
        image_message = bridge.cv2_to_imgmsg(image, encoding='mono8')
        image_message.header.seq = sequence
        image_message.header.stamp = stamp
        image_message.header.frame_id = 'usb_camera_optical_frame'
        info_pub.publish(camera_info(image_message.header))
        image_pub.publish(image_message)
        rate.sleep()
    rospy.sleep(0.5)

    with lock:
        pose_snapshot = list(poses)
        transform_snapshot = list(transforms)
        time_snapshot = list(processing_times)
    minimum_count = int(args.frames * 0.90)
    if len(pose_snapshot) < minimum_count or len(transform_snapshot) < minimum_count:
        raise RuntimeError('incomplete Pose/debug output')
    if len(time_snapshot) < minimum_count:
        raise RuntimeError('incomplete processing-time output')
    if not all(math.isfinite(item.data) and item.data >= 0.0 for item in time_snapshot):
        raise RuntimeError('processing time contains an invalid value')

    transform_by_stamp = {
        message.header.stamp.to_nsec(): message for message in transform_snapshot
    }
    for pose in pose_snapshot:
        debug = transform_by_stamp.get(pose.header.stamp.to_nsec())
        if debug is None or debug.header.frame_id != pose.header.frame_id:
            raise RuntimeError('Pose and debug transform headers do not match')
        matrix = np.asarray(debug.matrix, dtype=float).reshape(4, 4)
        translation = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ])
        quaternion = pose.pose.orientation
        norm = math.sqrt(
            quaternion.x ** 2 + quaternion.y ** 2 +
            quaternion.z ** 2 + quaternion.w ** 2
        )
        if not math.isclose(norm, 1.0, abs_tol=1e-8):
            raise RuntimeError('Pose quaternion is not normalized')
        if not np.allclose(matrix[:3, 3], translation, atol=1e-9):
            raise RuntimeError('Pose and debug translations differ')
        if not np.allclose(matrix[:3, :3], quaternion_matrix(quaternion), atol=1e-8):
            raise RuntimeError('Pose and debug rotations differ')
        if debug.marker_id != 33:
            raise RuntimeError('debug transform marker ID is not 33')

    print('VISION_REPLAY_PASS poses=%d processing=%d' % (
        len(pose_snapshot), len(time_snapshot)
    ))


if __name__ == '__main__':
    main()
