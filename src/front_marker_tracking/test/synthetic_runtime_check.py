#!/usr/bin/env python3

import math
import threading
import time

from front_marker_tracking.msg import HomogeneousTransform
from front_marker_tracking.transforms import (
    DEFAULT_BODY_R_CAMERA_OPTICAL,
    DEFAULT_BODY_T_CAMERA_OPTICAL,
    body_direction_to_camera,
    body_target_to_camera,
)
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget
import numpy as np
import rospy


FRONT_FACING_ROTATION = np.diag([1.0, -1.0, -1.0])


def main():
    rospy.init_node('front_marker_synthetic_runtime_check')
    lock = threading.Lock()
    commands = []
    filtered_errors = []
    preview_times = []

    def command_callback(message):
        with lock:
            commands.append(message)

    def preview_callback(_message):
        with lock:
            preview_times.append(rospy.get_time())

    def filtered_error_callback(message):
        with lock:
            filtered_errors.append(message)

    transform_pub = rospy.Publisher(
        '/front_marker_tracking/transform',
        HomogeneousTransform,
        queue_size=10,
    )
    rospy.Subscriber(
        '/front_marker_tracking/cmd_vel_flu',
        TwistStamped,
        command_callback,
        queue_size=200,
    )
    rospy.Subscriber(
        '/front_marker_tracking/setpoint_preview',
        PositionTarget,
        preview_callback,
        queue_size=200,
    )
    rospy.Subscriber(
        '/front_marker_tracking/error_flu_filtered',
        TwistStamped,
        filtered_error_callback,
        queue_size=200,
    )

    deadline = time.time() + 5.0
    while transform_pub.get_num_connections() < 1 and time.time() < deadline:
        rospy.sleep(0.02)
    if transform_pub.get_num_connections() < 1:
        raise RuntimeError('production tracking node did not subscribe')

    target_camera = body_target_to_camera(
        [0.8, 0.0, 0.0],
        DEFAULT_BODY_R_CAMERA_OPTICAL,
        DEFAULT_BODY_T_CAMERA_OPTICAL,
    )

    def message_for(body_error, yaw_error=0.0):
        camera_error = DEFAULT_BODY_R_CAMERA_OPTICAL.T.dot(
            np.asarray(body_error, dtype=np.float64)
        )
        transform = np.eye(4)
        body_forward_camera = body_direction_to_camera(
            [1.0, 0.0, 0.0], DEFAULT_BODY_R_CAMERA_OPTICAL
        )
        target_yaw = math.atan2(
            float(body_forward_camera[0]),
            float(body_forward_camera[2]),
        )
        marker_yaw = target_yaw - float(yaw_error)
        cosine = math.cos(marker_yaw)
        sine = math.sin(marker_yaw)
        rotation_y = np.array(
            [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]
        )
        transform[:3, :3] = rotation_y.dot(FRONT_FACING_ROTATION)
        transform[:3, 3] = target_camera + camera_error
        message = HomogeneousTransform()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'usb_camera_optical_frame'
        message.marker_id = 33
        message.matrix = transform.reshape(16).tolist()
        return message

    def run_stage(body_error, yaw_error=0.0, duration=0.6):
        with lock:
            start_index = len(commands)
        rate = rospy.Rate(20.0)
        end_time = time.time() + duration
        while time.time() < end_time:
            transform_pub.publish(message_for(body_error, yaw_error))
            rate.sleep()
        rospy.sleep(0.1)
        with lock:
            stage_commands = commands[start_index:]
        if not stage_commands:
            raise RuntimeError('no command received for stage %s' % body_error)
        return np.asarray(
            [
                [
                    message.twist.linear.x,
                    message.twist.linear.y,
                    message.twist.linear.z,
                    message.twist.angular.z,
                ]
                for message in stage_commands
            ],
            dtype=float,
        )

    zero = run_stage([0.0, 0.0, 0.0])
    positive_x = run_stage([1.0, 0.0, 0.0])
    negative_x = run_stage([-1.0, 0.0, 0.0])
    yz_limited = run_stage([0.0, 2.0, 2.0])
    z_deadband = run_stage([0.0, 0.0, 0.039])
    yaw_step = run_stage([0.0, 0.0, 0.0], yaw_error=0.4)

    if np.max(np.abs(zero[:, :3])) > 1e-9:
        raise RuntimeError('zero position error produced linear command')
    if not math.isclose(float(positive_x[-1, 0]), 0.2, abs_tol=1e-6):
        raise RuntimeError('positive X command did not reach +vx_limit')
    if not math.isclose(float(negative_x[-1, 0]), -0.2, abs_tol=1e-6):
        raise RuntimeError('negative X command did not reach -vx_limit')
    yz_norm = np.linalg.norm(yz_limited[-1, 1:3])
    if not math.isclose(float(yz_norm), 0.8, abs_tol=1e-6):
        raise RuntimeError('YZ command did not preserve v_limit')
    if abs(float(z_deadband[-1, 2])) > 1e-9:
        raise RuntimeError('Z error deadband did not force zero output')
    if not math.isclose(float(yaw_step[-1, 3]), 0.32, abs_tol=0.01):
        raise RuntimeError('filtered yaw step did not settle near kp_yaw * error')
    with lock:
        latest_filtered_yaw = filtered_errors[-1].twist.angular.z
    if not math.isclose(float(latest_filtered_yaw), 0.4, abs_tol=0.01):
        raise RuntimeError('filtered yaw error did not settle near the input')

    with lock:
        rate_times = list(preview_times)
    intervals = np.diff(rate_times)
    preview_rate = (len(rate_times) - 1) / (rate_times[-1] - rate_times[0])
    if not 19.0 <= preview_rate <= 21.0:
        raise RuntimeError('preview rate %.3fHz is outside 19-21Hz' % preview_rate)
    if float(np.max(intervals)) >= 0.1:
        raise RuntimeError('preview heartbeat gap reached 100ms')

    rospy.sleep(0.7)
    with lock:
        final_command = commands[-1]
    if any(
        abs(value) > 1e-9
        for value in (
            final_command.twist.linear.x,
            final_command.twist.linear.y,
            final_command.twist.linear.z,
            final_command.twist.angular.z,
        )
    ):
        raise RuntimeError('visual timeout did not restore zero command')
    if '/mavros/setpoint_raw/local' in dict(rospy.get_published_topics()):
        raise RuntimeError('safe deployment created a real setpoint publisher')

    print(
        'SYNTHETIC_PASS preview_hz=%.3f max_gap=%.4fs '
        'vx=(%.3f,%.3f) yz_norm=%.3f yaw_step=%.3f '
        'timeout_zero=true'
        % (
            preview_rate,
            float(np.max(intervals)),
            float(negative_x[-1, 0]),
            float(positive_x[-1, 0]),
            float(yz_norm),
            float(yaw_step[-1, 3]),
        )
    )


if __name__ == '__main__':
    main()
