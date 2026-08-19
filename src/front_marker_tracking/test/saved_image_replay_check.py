#!/usr/bin/env python3

import argparse
import threading

import cv2
from cv_bridge import CvBridge
from front_marker_tracking.msg import HomogeneousTransform
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image


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
    message.P = [
        741.67267,
        0.0,
        290.60259,
        0.0,
        0.0,
        741.44273,
        217.48395,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--frames', type=int, default=200)
    parser.add_argument('--rate', type=float, default=20.0)
    args = parser.parse_args()

    image = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
    if image is None or image.shape != (480, 640):
        raise RuntimeError('expected a readable 640x480 grayscale image')

    rospy.init_node('saved_nested_marker_replay_check')
    bridge = CvBridge()
    image_pub = rospy.Publisher('/test_saved/image_raw', Image, queue_size=2)
    info_pub = rospy.Publisher('/test_saved/camera_info', CameraInfo, queue_size=2)
    lock = threading.Lock()
    data = {
        key: []
        for key in (
            'transform',
            'camera_pose',
            'error',
            'command',
            'preview',
        )
    }

    def callback(key):
        def append(message):
            with lock:
                data[key].append(message)

        return append

    rospy.Subscriber(
        '/front_marker_tracking/transform',
        HomogeneousTransform,
        callback('transform'),
        queue_size=100,
    )
    rospy.Subscriber(
        '/front_marker_tracking/marker_pose_camera',
        PoseStamped,
        callback('camera_pose'),
        queue_size=100,
    )
    rospy.Subscriber(
        '/front_marker_tracking/error_flu',
        TwistStamped,
        callback('error'),
        queue_size=100,
    )
    rospy.Subscriber(
        '/front_marker_tracking/cmd_vel_flu',
        TwistStamped,
        callback('command'),
        queue_size=200,
    )
    rospy.Subscriber(
        '/front_marker_tracking/setpoint_preview',
        PositionTarget,
        callback('preview'),
        queue_size=200,
    )

    deadline = rospy.get_time() + 5.0
    while image_pub.get_num_connections() == 0 and rospy.get_time() < deadline:
        rospy.sleep(0.02)
    if image_pub.get_num_connections() == 0:
        raise RuntimeError('detector did not subscribe to replay image topic')

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
        snapshot = {key: list(value) for key, value in data.items()}
    minimum_count = int(args.frames * 0.95)
    for key in ('transform', 'camera_pose', 'error'):
        if len(snapshot[key]) < minimum_count:
            raise RuntimeError('%s count %d < %d' % (
                key, len(snapshot[key]), minimum_count
            ))
    if not all(message.marker_id == 33 for message in snapshot['transform']):
        raise RuntimeError('detector selected a marker other than ID33')

    matrices = np.asarray(
        [message.matrix for message in snapshot['transform']], dtype=float
    )
    if matrices.shape[1] != 16 or not np.isfinite(matrices).all():
        raise RuntimeError('transform matrices are not finite 4x4 values')
    stamps = np.asarray(
        [message.header.stamp.to_sec() for message in snapshot['transform']]
    )
    if not np.all(np.diff(stamps) > 0.0):
        raise RuntimeError('transform source stamps are not strictly increasing')
    pose_rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    if not 19.0 <= pose_rate <= 21.0:
        raise RuntimeError('pose rate %.3fHz is outside 19-21Hz' % pose_rate)

    error_stamps = {
        message.header.stamp.to_nsec() for message in snapshot['error']
    }
    if not all(
        message.header.stamp.to_nsec() in error_stamps
        for message in snapshot['transform']
    ):
        raise RuntimeError('transform and FLU error source stamps do not align')
    if (
        len(snapshot['command']) < minimum_count
        or len(snapshot['preview']) < minimum_count
    ):
        raise RuntimeError('20Hz command/preview heartbeat is incomplete')

    commands = np.asarray(
        [
            [
                message.twist.linear.x,
                message.twist.linear.y,
                message.twist.linear.z,
                message.twist.angular.z,
            ]
            for message in snapshot['command']
        ]
    )
    if not np.isfinite(commands).all():
        raise RuntimeError('command contains non-finite values')
    if np.max(np.abs(commands[:, 0])) > 0.200001:
        raise RuntimeError('X command exceeded vx_limit')
    if np.max(np.linalg.norm(commands[:, 1:3], axis=1)) > 0.800001:
        raise RuntimeError('YZ command exceeded v_limit')
    if np.max(np.abs(commands[:, 3])) > 0.800001:
        raise RuntimeError('yaw command exceeded yaw_rate_limit')
    if '/mavros/setpoint_raw/local' in dict(rospy.get_published_topics()):
        raise RuntimeError('preview-only controller created a real setpoint publisher')

    print(
        'REPLAY_PASS frames=%d pose_hz=%.3f camera_pose=%d '
        'error=%d command=%d preview=%d max_abs_cmd=%s'
        % (
            len(stamps),
            pose_rate,
            len(snapshot['camera_pose']),
            len(snapshot['error']),
            len(snapshot['command']),
            len(snapshot['preview']),
            np.max(np.abs(commands), axis=0).tolist(),
        )
    )


if __name__ == '__main__':
    main()
