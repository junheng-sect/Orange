"""ROS Noetic controller for forward USB-camera Marker alignment."""

import math
import threading

from front_marker_tracking.msg import HomogeneousTransform
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
import numpy as np
import rospy
from std_msgs.msg import Float64, String

from front_marker_tracking.control import BodyVelocityController
from front_marker_tracking.mavros_interface import (
    body_flu_setpoint_components,
    velocity_only_type_mask,
)
from front_marker_tracking.transforms import (
    DEFAULT_BODY_R_CAMERA_OPTICAL,
    DEFAULT_BODY_T_CAMERA_OPTICAL,
    body_direction_to_camera,
    body_target_to_camera,
    camera_velocity_to_body,
    rotation_to_quaternion,
    row_major_matrix,
    tracking_errors,
    validate_rotation,
    validate_vector3,
)


class FrontMarkerTrackingNode(object):
    """Publish visual command previews and optionally gated MAVROS setpoints."""

    def __init__(self):
        self.lock = threading.RLock()
        self._read_parameters()

        self.state = State()
        self.latest_state_receive_time = None
        self.latest_position_error = None
        self.latest_position_error_body = None
        self.latest_yaw_error = 0.0
        self.latest_filtered_yaw_error = 0.0
        self.latest_source_stamp = None
        self.latest_receive_time = None
        self.last_pid_source_stamp = None
        self.last_accepted_source_stamp = None
        self.aligned_start_time = None
        self.latest_camera_velocity = np.zeros(3, dtype=np.float64)
        self.latest_body_velocity = np.zeros(3, dtype=np.float64)
        self.latest_yaw_rate = 0.0
        self.last_observation_issue = '尚未收到目标 Marker 位姿'
        self.last_status = 'PREVIEW_ONLY | 等待目标 Marker'

        self.body_controller = BodyVelocityController(
            self.kp_x,
            self.ki_x,
            self.kd_x,
            self.kp_yz,
            self.ki_yz,
            self.kd_yz,
            self.kp_yaw,
            self.ki_yaw,
            self.kd_yaw,
            self.vx_limit,
            self.v_limit,
            self.yaw_rate_limit,
            self.velocity_deadband,
            self.z_error_deadband,
            self.yaw_error_deadband,
            self.yaw_lpf_tau,
            self.enable_yaw_control,
        )

        self.marker_pose_pub = rospy.Publisher(
            '/front_marker_tracking/marker_pose_camera', PoseStamped, queue_size=10
        )
        self.position_error_pub = rospy.Publisher(
            '/front_marker_tracking/position_error_camera', Vector3Stamped, queue_size=10
        )
        self.yaw_error_pub = rospy.Publisher(
            '/front_marker_tracking/yaw_error', Float64, queue_size=10
        )
        self.body_error_pub = rospy.Publisher(
            '/front_marker_tracking/error_flu', TwistStamped, queue_size=10
        )
        self.filtered_body_error_pub = rospy.Publisher(
            '/front_marker_tracking/error_flu_filtered',
            TwistStamped,
            queue_size=10,
        )
        self.camera_command_pub = rospy.Publisher(
            '/front_marker_tracking/cmd_vel_camera', TwistStamped, queue_size=10
        )
        self.body_command_pub = rospy.Publisher(
            '/front_marker_tracking/cmd_vel_flu', TwistStamped, queue_size=10
        )
        self.preview_pub = rospy.Publisher(
            self.preview_topic, PositionTarget, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            '/front_marker_tracking/status', String, queue_size=10, latch=True
        )

        # The real MAVROS publisher does not exist at all in the default preview mode.
        self.setpoint_pub = None
        if self.enable_setpoint_output:
            self.setpoint_pub = rospy.Publisher(
                self.setpoint_topic, PositionTarget, queue_size=10
            )
            rospy.logwarn(
                '真实 MAVROS setpoint 输出已显式启用；非零输出仍强制要求 FCU connected + armed + OFFBOARD + 新鲜视觉'
            )
        else:
            rospy.logwarn('PREVIEW_ONLY：不会创建 %s 发布者', self.setpoint_topic)

        rospy.Subscriber(
            self.transform_topic,
            HomogeneousTransform,
            self.transform_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.state_topic,
            State,
            self.state_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        rospy.Timer(
            rospy.Duration(1.0 / self.setpoint_publish_rate_hz),
            self.control_loop,
        )
        rospy.Timer(rospy.Duration(1.0), self.status_loop)

        rospy.loginfo(
            'front_marker_tracking_node started | transform=%s | target=ID%d '
            'body=(%.3f, %.3f, %.3f)m | body_t_camera=(%.4f, %.4f, %.4f)m | '
            'equivalent_camera=(%.6f, %.6f, %.6f)m | yaw_target_camera=%.3fdeg | '
            'yaw_pid=(%.3f, %.3f, %.3f) | yaw_lpf_tau=%.3fs | '
            'yaw_control=%s | setpoint_rate=%.1fHz | setpoint_output=%s',
            self.transform_topic,
            self.target_marker_id,
            self.target_xyz_body[0],
            self.target_xyz_body[1],
            self.target_xyz_body[2],
            self.body_t_camera_optical[0],
            self.body_t_camera_optical[1],
            self.body_t_camera_optical[2],
            self.target_xyz_camera[0],
            self.target_xyz_camera[1],
            self.target_xyz_camera[2],
            math.degrees(self.target_yaw_camera),
            self.kp_yaw,
            self.ki_yaw,
            self.kd_yaw,
            self.yaw_lpf_tau,
            'ON' if self.enable_yaw_control else 'OFF',
            self.setpoint_publish_rate_hz,
            self.enable_setpoint_output,
        )

    def _read_parameters(self):
        get = rospy.get_param
        self.transform_topic = str(
            get('~transform_topic', '/front_marker_tracking/transform')
        )
        self.state_topic = str(get('~state_topic', '/mavros/state'))
        self.setpoint_topic = str(
            get('~setpoint_raw_topic', '/mavros/setpoint_raw/local')
        )
        self.preview_topic = str(
            get('~setpoint_preview_topic', '/front_marker_tracking/setpoint_preview')
        )
        self.base_frame = str(get('~base_frame', 'base_link'))
        self.camera_frame = str(
            get('~camera_optical_frame', 'usb_camera_optical_frame')
        )
        self.target_marker_id = int(get('~target_marker_id', 33))

        legacy_target_parameters = [
            name
            for name in ('target_x', 'target_y', 'target_distance_m')
            if rospy.has_param('~' + name)
        ]
        if legacy_target_parameters:
            raise ValueError(
                'legacy camera-frame target parameters are not supported: %s; '
                'use target_xyz_body instead'
                % ', '.join(legacy_target_parameters)
            )

        legacy_control_parameters = [
            name
            for name in (
                'kp_xy',
                'ki_xy',
                'kd_xy',
                'kp_z',
                'ki_z',
                'kd_z',
                'max_linear_speed',
                'max_yaw_rate',
                'yaw_rate_deadband',
                'control_rate_hz',
            )
            if rospy.has_param('~' + name)
            and str(rospy.get_param('~' + name)) != '__UNSET__'
        ]
        if legacy_control_parameters:
            raise ValueError(
                'legacy camera-axis control parameters are not supported: %s; '
                'use kp_x/kp_yz, vx_limit/v_limit, yaw_rate_limit and '
                'setpoint_publish_rate_hz'
                % ', '.join(legacy_control_parameters)
            )

        rotation_values = get(
            '~body_R_camera_optical',
            DEFAULT_BODY_R_CAMERA_OPTICAL.reshape(9).tolist(),
        )
        if len(rotation_values) != 9:
            raise ValueError('body_R_camera_optical must contain 9 row-major values')
        self.body_r_camera_optical = validate_rotation(
            np.asarray(rotation_values, dtype=np.float64).reshape(3, 3)
        )
        self.body_t_camera_optical = validate_vector3(
            get(
                '~body_t_camera_optical',
                DEFAULT_BODY_T_CAMERA_OPTICAL.tolist(),
            ),
            'body_t_camera_optical',
        )
        self.target_xyz_body = validate_vector3(
            get('~target_xyz_body', [0.8, 0.0, 0.0]),
            'target_xyz_body',
        )
        self.target_xyz_camera = body_target_to_camera(
            self.target_xyz_body,
            self.body_r_camera_optical,
            self.body_t_camera_optical,
        )
        self.target_facing_direction_camera = body_direction_to_camera(
            [1.0, 0.0, 0.0], self.body_r_camera_optical
        )
        self.target_yaw_camera = math.atan2(
            float(self.target_facing_direction_camera[0]),
            float(self.target_facing_direction_camera[2]),
        )

        numeric_defaults = {
            'kp_x': 0.50,
            'ki_x': 0.0,
            'kd_x': 0.03,
            'kp_yz': 0.60,
            'ki_yz': 0.0,
            'kd_yz': 0.08,
            'kp_yaw': 0.40,
            'ki_yaw': 0.0,
            'kd_yaw': 0.0,
            'yaw_lpf_tau': 0.10,
            'v_limit': 0.80,
            'vx_limit': 0.20,
            'yaw_rate_limit': 0.80,
            'velocity_deadband': 0.03,
            'z_error_deadband': 0.04,
            'yaw_error_deadband': 0.06,
            'setpoint_publish_rate_hz': 20.0,
            'pose_timeout_sec': 0.5,
            'state_timeout_sec': 1.0,
            'future_stamp_tolerance_sec': 0.1,
            'lateral_tolerance_m': 0.08,
            'vertical_tolerance_m': 0.08,
            'distance_tolerance_m': 0.12,
            'yaw_tolerance_deg': 6.0,
            'aligned_hold_sec': 1.0,
        }
        for name, default in numeric_defaults.items():
            setattr(self, name, float(get('~' + name, default)))

        self.enable_setpoint_output = bool(
            get('~enable_setpoint_output', False)
        )
        self.enable_yaw_control = bool(get('~enable_yaw_control', False))
        self.yaw_tolerance_rad = math.radians(self.yaw_tolerance_deg)

        for name in numeric_defaults:
            if not math.isfinite(getattr(self, name)):
                raise ValueError('%s must be finite' % name)
        if self.target_marker_id < 0:
            raise ValueError('target_marker_id must be non-negative')
        if self.target_xyz_camera[2] <= 0.0:
            raise ValueError(
                'target_xyz_body maps behind the camera; equivalent camera z must be positive'
            )
        if self.setpoint_publish_rate_hz <= 0.0:
            raise ValueError('setpoint_publish_rate_hz must be positive')
        if self.pose_timeout_sec <= 0.0 or self.state_timeout_sec <= 0.0:
            raise ValueError('pose_timeout_sec and state_timeout_sec must be positive')
        if self.future_stamp_tolerance_sec < 0.0:
            raise ValueError('future_stamp_tolerance_sec must be non-negative')
        if (
            self.v_limit < 0.0
            or self.vx_limit < 0.0
            or self.yaw_rate_limit < 0.0
            or self.velocity_deadband < 0.0
            or self.z_error_deadband < 0.0
            or self.yaw_error_deadband < 0.0
            or self.yaw_lpf_tau < 0.0
        ):
            raise ValueError('speed limits, deadbands and yaw_lpf_tau must be non-negative')

    def state_callback(self, message):
        with self.lock:
            self.state = message
            self.latest_state_receive_time = rospy.Time.now()

    def invalidate_observation(self, reason):
        self.latest_position_error = None
        self.latest_position_error_body = None
        self.latest_yaw_error = 0.0
        self.latest_filtered_yaw_error = 0.0
        self.latest_source_stamp = None
        self.latest_receive_time = None
        self.last_observation_issue = reason
        self.reset_control()

    def transform_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            if int(message.marker_id) != self.target_marker_id:
                self.invalidate_observation(
                    '收到非目标 Marker ID%d，等待 ID%d'
                    % (int(message.marker_id), self.target_marker_id)
                )
                return
            if message.header.stamp == rospy.Time():
                self.invalidate_observation('拒绝零时间戳 Marker 位姿')
                return

            if self.last_accepted_source_stamp is not None:
                if message.header.stamp == self.last_accepted_source_stamp:
                    rospy.logwarn_throttle(
                        5.0,
                        '忽略重复 Marker 时间戳；保持最近一次 PID 指令',
                    )
                    return
                if message.header.stamp < self.last_accepted_source_stamp:
                    self.invalidate_observation('拒绝倒退时间戳 Marker 位姿')
                    return

            source_age = (now - message.header.stamp).to_sec()
            if source_age > self.pose_timeout_sec:
                self.invalidate_observation(
                    '收到时已过期的 Marker 位姿，source_age=%.3fs' % source_age
                )
                return
            if source_age < -self.future_stamp_tolerance_sec:
                self.invalidate_observation(
                    '拒绝未来时间戳 Marker 位姿，source_age=%.3fs' % source_age
                )
                return

            try:
                camera_t_marker = row_major_matrix(message.matrix)
                position_error, yaw_error, _ = tracking_errors(
                    camera_t_marker,
                    self.target_xyz_camera,
                    self.target_facing_direction_camera,
                )
            except ValueError as exception:
                self.invalidate_observation('拒绝非法齐次矩阵: %s' % exception)
                return

            position_error_body = camera_velocity_to_body(
                position_error, self.body_r_camera_optical
            )
            if self.last_pid_source_stamp is None:
                self.reset_pid_state()
                dt = 1.0 / self.setpoint_publish_rate_hz
            else:
                dt = (message.header.stamp - self.last_pid_source_stamp).to_sec()
                if (
                    not math.isfinite(dt)
                    or dt <= 0.0
                    or dt > self.pose_timeout_sec
                ):
                    self.reset_pid_state()
                    dt = 1.0 / self.setpoint_publish_rate_hz

            self.latest_position_error = position_error
            self.latest_position_error_body = position_error_body
            self.latest_yaw_error = yaw_error
            self.latest_source_stamp = message.header.stamp
            self.latest_receive_time = now
            self.last_pid_source_stamp = message.header.stamp
            self.last_accepted_source_stamp = message.header.stamp
            self.last_observation_issue = ''
            (
                self.latest_camera_velocity,
                self.latest_body_velocity,
                self.latest_yaw_rate,
                self.latest_filtered_yaw_error,
            ) = self.compute_command(position_error_body, yaw_error, dt)
            self.publish_observation(
                message,
                camera_t_marker,
                position_error,
                position_error_body,
                yaw_error,
                self.latest_filtered_yaw_error,
            )

    def publish_observation(
        self,
        source,
        camera_t_marker,
        position_error,
        position_error_body,
        yaw_error,
        filtered_yaw_error,
    ):
        frame_id = source.header.frame_id or self.camera_frame
        quaternion = rotation_to_quaternion(camera_t_marker[:3, :3])

        pose = PoseStamped()
        pose.header = source.header
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(camera_t_marker[0, 3])
        pose.pose.position.y = float(camera_t_marker[1, 3])
        pose.pose.position.z = float(camera_t_marker[2, 3])
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion
        self.marker_pose_pub.publish(pose)

        error_message = Vector3Stamped()
        error_message.header = pose.header
        error_message.vector.x = float(position_error[0])
        error_message.vector.y = float(position_error[1])
        error_message.vector.z = float(position_error[2])
        self.position_error_pub.publish(error_message)

        self.yaw_error_pub.publish(Float64(data=float(yaw_error)))

        body_error = TwistStamped()
        body_error.header = source.header
        body_error.header.frame_id = self.base_frame
        body_error.twist.linear.x = float(position_error_body[0])
        body_error.twist.linear.y = float(position_error_body[1])
        body_error.twist.linear.z = float(position_error_body[2])
        body_error.twist.angular.z = float(yaw_error)
        self.body_error_pub.publish(body_error)

        filtered_body_error = TwistStamped()
        filtered_body_error.header = source.header
        filtered_body_error.header.frame_id = self.base_frame
        filtered_body_error.twist.linear.x = float(position_error_body[0])
        filtered_body_error.twist.linear.y = float(position_error_body[1])
        filtered_body_error.twist.linear.z = float(position_error_body[2])
        filtered_body_error.twist.angular.z = float(filtered_yaw_error)
        self.filtered_body_error_pub.publish(filtered_body_error)

    def observation_is_fresh(self, now):
        if (
            self.latest_position_error is None
            or self.latest_source_stamp is None
            or self.latest_receive_time is None
        ):
            return False
        source_age = (now - self.latest_source_stamp).to_sec()
        receive_age = (now - self.latest_receive_time).to_sec()
        return (
            -self.future_stamp_tolerance_sec <= source_age <= self.pose_timeout_sec
            and 0.0 <= receive_age <= self.pose_timeout_sec
        )

    def state_is_fresh(self, now):
        if self.latest_state_receive_time is None:
            return False
        age = (now - self.latest_state_receive_time).to_sec()
        return 0.0 <= age <= self.state_timeout_sec

    def reset_pid_state(self):
        self.body_controller.reset()

    def reset_control(self):
        self.reset_pid_state()
        self.last_pid_source_stamp = None
        self.latest_camera_velocity = np.zeros(3, dtype=np.float64)
        self.latest_body_velocity = np.zeros(3, dtype=np.float64)
        self.latest_yaw_rate = 0.0
        self.aligned_start_time = None

    def compute_command(self, position_error_body, yaw_error, dt):
        (
            body_velocity_values,
            yaw_rate,
            filtered_yaw_error,
        ) = self.body_controller.update(
            position_error_body, yaw_error, dt
        )
        body_velocity = np.asarray(body_velocity_values, dtype=np.float64)
        camera_velocity = self.body_r_camera_optical.T.dot(body_velocity)
        return camera_velocity, body_velocity, yaw_rate, filtered_yaw_error

    def update_alignment_state(self, now):
        error = self.latest_position_error_body
        aligned = (
            abs(float(error[0])) <= self.distance_tolerance_m
            and abs(float(error[1])) <= self.lateral_tolerance_m
            and abs(float(error[2])) <= self.vertical_tolerance_m
            and (
                not self.enable_yaw_control
                or abs(float(self.latest_filtered_yaw_error))
                <= self.yaw_tolerance_rad
            )
        )
        if not aligned:
            self.aligned_start_time = None
            return 'ALIGN'
        if self.aligned_start_time is None:
            self.aligned_start_time = now
            return 'ALIGN'
        held = (now - self.aligned_start_time).to_sec()
        return 'ALIGNED' if held >= self.aligned_hold_sec else 'ALIGN'

    def actual_gate_reason(self, now, visual_fresh):
        if not self.state_is_fresh(now) or not self.state.connected:
            return 'WAIT_FCU'
        if not self.state.armed:
            return 'WAIT_ARM'
        if self.state.mode != 'OFFBOARD':
            return 'WAIT_OFFBOARD'
        if not visual_fresh:
            return 'WAIT_MARKER'
        return None

    def control_loop(self, _event):
        with self.lock:
            now = rospy.Time.now()
            visual_fresh = self.observation_is_fresh(now)
            if visual_fresh:
                camera_velocity = self.latest_camera_velocity.copy()
                body_velocity = self.latest_body_velocity.copy()
                yaw_rate = self.latest_yaw_rate
                nav_state = self.update_alignment_state(now)
            else:
                if self.latest_position_error is not None:
                    self.invalidate_observation('视觉超时')
                camera_velocity = np.zeros(3, dtype=np.float64)
                body_velocity = np.zeros(3, dtype=np.float64)
                yaw_rate = 0.0
                nav_state = 'WAIT_MARKER'

            actual_allowed = False
            gate_reason = None
            if self.enable_setpoint_output:
                gate_reason = self.actual_gate_reason(now, visual_fresh)
                actual_allowed = gate_reason is None

            self.publish_commands(
                now,
                camera_velocity,
                body_velocity,
                yaw_rate,
                actual_allowed,
            )

            if visual_fresh:
                error_baselink = self.latest_position_error_body
                preview_text = (
                    '%s ID%d | yaw_control=%s | '
                    'error_baselink=(%.3f, %.3f, %.3f)m '
                    'yaw_raw=%.2fdeg yaw_lpf=%.2fdeg | '
                    'preview_flu=(%.3f, %.3f, %.3f, %.3f)'
                    % (
                        nav_state,
                        self.target_marker_id,
                        'ON' if self.enable_yaw_control else 'OFF',
                        error_baselink[0],
                        error_baselink[1],
                        error_baselink[2],
                        math.degrees(self.latest_yaw_error),
                        math.degrees(self.latest_filtered_yaw_error),
                        body_velocity[0],
                        body_velocity[1],
                        body_velocity[2],
                        yaw_rate,
                    )
                )
            else:
                preview_text = '%s | yaw_control=%s | %s | preview=zero' % (
                    nav_state,
                    'ON' if self.enable_yaw_control else 'OFF',
                    self.last_observation_issue or '视觉超时',
                )

            if not self.enable_setpoint_output:
                self.last_status = 'PREVIEW_ONLY | ' + preview_text
            elif actual_allowed:
                self.last_status = 'SETPOINT_ACTIVE | ' + preview_text
            else:
                self.last_status = '%s | actual=zero | %s' % (
                    gate_reason,
                    preview_text,
                )

    def make_setpoint(self, stamp, body_velocity, yaw_rate):
        vx, vy, vz, wz = body_flu_setpoint_components(
            body_velocity[0], body_velocity[1], body_velocity[2], yaw_rate
        )
        message = PositionTarget()
        message.header.stamp = stamp
        message.coordinate_frame = PositionTarget.FRAME_BODY_NED
        message.type_mask = velocity_only_type_mask()
        message.velocity.x = vx
        message.velocity.y = vy
        message.velocity.z = vz
        message.yaw_rate = wz
        return message

    def publish_commands(
        self,
        stamp,
        camera_velocity,
        body_velocity,
        yaw_rate,
        actual_allowed,
    ):
        if not self.enable_yaw_control:
            yaw_rate = 0.0

        camera_command = TwistStamped()
        camera_command.header.stamp = stamp
        camera_command.header.frame_id = self.camera_frame
        camera_command.twist.linear.x = float(camera_velocity[0])
        camera_command.twist.linear.y = float(camera_velocity[1])
        camera_command.twist.linear.z = float(camera_velocity[2])
        self.camera_command_pub.publish(camera_command)

        body_command = TwistStamped()
        body_command.header.stamp = stamp
        body_command.header.frame_id = self.base_frame
        body_command.twist.linear.x = float(body_velocity[0])
        body_command.twist.linear.y = float(body_velocity[1])
        body_command.twist.linear.z = float(body_velocity[2])
        body_command.twist.angular.z = float(yaw_rate)
        self.body_command_pub.publish(body_command)

        preview = self.make_setpoint(stamp, body_velocity, yaw_rate)
        self.preview_pub.publish(preview)

        if self.setpoint_pub is not None:
            if actual_allowed:
                actual = preview
            else:
                actual = self.make_setpoint(stamp, np.zeros(3), 0.0)
            self.setpoint_pub.publish(actual)

    def status_loop(self, _event):
        with self.lock:
            text = self.last_status
        self.status_pub.publish(String(data=text))
        rospy.loginfo(text)


def main():
    rospy.init_node('front_marker_tracking_node')
    FrontMarkerTrackingNode()
    rospy.spin()
