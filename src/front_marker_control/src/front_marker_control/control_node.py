"""Pose-driven body-velocity controller with gated MAVROS output."""

import math
import threading

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
import rospy
from std_msgs.msg import Float64, String
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped conversions
import tf2_ros

from front_marker_control.control import BodyVelocityController, wrap_angle
from front_marker_control.geometry import (
    finite_vector3,
    marker_heading_body,
    normalize_quaternion,
    position_error_body,
)
from front_marker_control.mavros_interface import (
    body_flu_setpoint_components,
    velocity_only_type_mask,
)


class FrontMarkerControlNode(object):
    """Transform standard Marker poses and publish safe body velocity commands."""

    def __init__(self):
        self.lock = threading.RLock()
        self._read_parameters()

        self.state = State()
        self.latest_state_receive_time = None
        self.latest_position_error = None
        self.latest_yaw_error = 0.0
        self.latest_filtered_yaw_error = 0.0
        self.latest_source_stamp = None
        self.latest_receive_time = None
        self.last_pid_source_stamp = None
        self.last_accepted_source_stamp = None
        self.aligned_start_time = None
        self.latest_body_velocity = (0.0, 0.0, 0.0)
        self.latest_yaw_rate = 0.0
        self.last_observation_issue = '尚未收到 Marker Pose'
        self.last_status = 'PREVIEW_ONLY | WAIT_POSE'

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

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.position_error_pub = rospy.Publisher(
            self.position_error_topic, Vector3Stamped, queue_size=10
        )
        self.yaw_error_pub = rospy.Publisher(
            self.yaw_error_topic, Float64, queue_size=10
        )
        self.command_pub = rospy.Publisher(
            self.command_topic, TwistStamped, queue_size=10
        )
        self.preview_pub = rospy.Publisher(
            self.preview_topic, PositionTarget, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=True
        )

        self.setpoint_pub = None
        if self.enable_setpoint_output:
            self.setpoint_pub = rospy.Publisher(
                self.setpoint_topic, PositionTarget, queue_size=10
            )
            rospy.logwarn(
                '真实 MAVROS setpoint 已启用；非零输出仍要求 connected + armed + OFFBOARD + 新鲜 Pose/TF'
            )
        else:
            rospy.logwarn('PREVIEW_ONLY：不会创建 %s 发布者', self.setpoint_topic)

        rospy.Subscriber(
            self.pose_topic,
            PoseStamped,
            self.pose_callback,
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
            'front_marker_control_node started | pose=%s | base=%s | '
            'target=(%.3f, %.3f, %.3f)m | yaw_control=%s | '
            'setpoint_rate=%.1fHz | setpoint_output=%s',
            self.pose_topic,
            self.base_frame,
            self.target_xyz_body[0],
            self.target_xyz_body[1],
            self.target_xyz_body[2],
            'ON' if self.enable_yaw_control else 'OFF',
            self.setpoint_publish_rate_hz,
            self.enable_setpoint_output,
        )

    def _read_parameters(self):
        get = rospy.get_param
        self.pose_topic = str(get('~pose_topic', '/front_marker/pose'))
        self.state_topic = str(get('~state_topic', '/mavros/state'))
        self.setpoint_topic = str(
            get('~setpoint_raw_topic', '/mavros/setpoint_raw/local')
        )
        self.position_error_topic = str(
            get('~position_error_topic', '/front_marker_control/position_error')
        )
        self.yaw_error_topic = str(
            get('~yaw_error_topic', '/front_marker_control/yaw_error')
        )
        self.command_topic = str(
            get('~command_topic', '/front_marker_control/cmd_vel')
        )
        self.preview_topic = str(
            get('~setpoint_preview_topic', '/front_marker_control/setpoint_preview')
        )
        self.status_topic = str(
            get('~status_topic', '/front_marker_control/status')
        )
        self.base_frame = str(get('~base_frame', 'base_link'))
        if not self.base_frame:
            raise ValueError('base_frame must not be empty')
        self.target_xyz_body = finite_vector3(
            get('~target_xyz_body', [0.8, 0.0, 0.0]),
            'target_xyz_body',
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
            'tf_lookup_timeout_sec': 0.05,
            'lateral_tolerance_m': 0.08,
            'vertical_tolerance_m': 0.08,
            'distance_tolerance_m': 0.12,
            'yaw_tolerance_deg': 6.0,
            'aligned_hold_sec': 1.0,
        }
        for name, default in numeric_defaults.items():
            setattr(self, name, float(get('~' + name, default)))

        self.enable_setpoint_output = bool(get('~enable_setpoint_output', False))
        self.enable_yaw_control = bool(get('~enable_yaw_control', False))
        self.yaw_tolerance_rad = math.radians(self.yaw_tolerance_deg)

        for name in numeric_defaults:
            if not math.isfinite(getattr(self, name)):
                raise ValueError('%s must be finite' % name)
        if self.setpoint_publish_rate_hz <= 0.0:
            raise ValueError('setpoint_publish_rate_hz must be positive')
        if self.pose_timeout_sec <= 0.0 or self.state_timeout_sec <= 0.0:
            raise ValueError('pose_timeout_sec and state_timeout_sec must be positive')
        if self.future_stamp_tolerance_sec < 0.0 or self.tf_lookup_timeout_sec < 0.0:
            raise ValueError('timestamp and TF tolerances must be non-negative')
        if any(value < 0.0 for value in (
                self.v_limit,
                self.vx_limit,
                self.yaw_rate_limit,
                self.velocity_deadband,
                self.z_error_deadband,
                self.yaw_error_deadband,
                self.yaw_lpf_tau,
                self.lateral_tolerance_m,
                self.vertical_tolerance_m,
                self.distance_tolerance_m,
                self.yaw_tolerance_deg,
                self.aligned_hold_sec,
        )):
            raise ValueError('limits, deadbands and tolerances must be non-negative')

    def state_callback(self, message):
        with self.lock:
            self.state = message
            self.latest_state_receive_time = rospy.Time.now()

    def invalidate_observation(self, reason):
        self.latest_position_error = None
        self.latest_yaw_error = 0.0
        self.latest_filtered_yaw_error = 0.0
        self.latest_source_stamp = None
        self.latest_receive_time = None
        self.last_observation_issue = reason
        self.reset_control()

    def normalized_pose(self, message):
        if not message.header.frame_id:
            raise ValueError('pose frame_id must not be empty')
        x, y, z, w = normalize_quaternion(message.pose.orientation)
        normalized = PoseStamped()
        normalized.header = message.header
        normalized.pose.position.x = message.pose.position.x
        normalized.pose.position.y = message.pose.position.y
        normalized.pose.position.z = message.pose.position.z
        normalized.pose.orientation.x = x
        normalized.pose.orientation.y = y
        normalized.pose.orientation.z = z
        normalized.pose.orientation.w = w
        return normalized

    def pose_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            if message.header.stamp == rospy.Time():
                self.invalidate_observation('拒绝零时间戳 Pose')
                return
            if self.last_accepted_source_stamp is not None:
                if message.header.stamp == self.last_accepted_source_stamp:
                    self.invalidate_observation('拒绝重复时间戳 Pose')
                    return
                if message.header.stamp < self.last_accepted_source_stamp:
                    self.invalidate_observation('拒绝倒退时间戳 Pose')
                    return

            source_age = (now - message.header.stamp).to_sec()
            if source_age > self.pose_timeout_sec:
                self.invalidate_observation(
                    '收到时已过期的 Pose，source_age=%.3fs' % source_age
                )
                return
            if source_age < -self.future_stamp_tolerance_sec:
                self.invalidate_observation(
                    '拒绝未来时间戳 Pose，source_age=%.3fs' % source_age
                )
                return

            try:
                normalized = self.normalized_pose(message)
                pose_body = self.tf_buffer.transform(
                    normalized,
                    self.base_frame,
                    rospy.Duration(self.tf_lookup_timeout_sec),
                )
                error = position_error_body(pose_body.pose, self.target_xyz_body)
                yaw_error = wrap_angle(marker_heading_body(pose_body.pose.orientation))
            except ValueError as exception:
                self.invalidate_observation('拒绝非法 Pose: %s' % exception)
                return
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exception:
                self.invalidate_observation('WAIT_TF: %s' % exception)
                return

            if self.last_pid_source_stamp is None:
                self.reset_pid_state()
                dt = 1.0 / self.setpoint_publish_rate_hz
            else:
                dt = (message.header.stamp - self.last_pid_source_stamp).to_sec()
                if not math.isfinite(dt) or dt <= 0.0 or dt > self.pose_timeout_sec:
                    self.reset_pid_state()
                    dt = 1.0 / self.setpoint_publish_rate_hz

            (
                self.latest_body_velocity,
                self.latest_yaw_rate,
                self.latest_filtered_yaw_error,
            ) = self.body_controller.update(error, yaw_error, dt)
            self.latest_position_error = error
            self.latest_yaw_error = yaw_error
            self.latest_source_stamp = message.header.stamp
            self.latest_receive_time = now
            self.last_pid_source_stamp = message.header.stamp
            self.last_accepted_source_stamp = message.header.stamp
            self.last_observation_issue = ''
            self.publish_errors(message.header.stamp, error, yaw_error)

    def publish_errors(self, stamp, error, yaw_error):
        message = Vector3Stamped()
        message.header.stamp = stamp
        message.header.frame_id = self.base_frame
        message.vector.x = float(error[0])
        message.vector.y = float(error[1])
        message.vector.z = float(error[2])
        self.position_error_pub.publish(message)
        self.yaw_error_pub.publish(Float64(data=float(yaw_error)))

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
        self.latest_body_velocity = (0.0, 0.0, 0.0)
        self.latest_yaw_rate = 0.0
        self.aligned_start_time = None

    def update_alignment_state(self, now):
        error = self.latest_position_error
        aligned = (
            abs(float(error[0])) <= self.distance_tolerance_m
            and abs(float(error[1])) <= self.lateral_tolerance_m
            and abs(float(error[2])) <= self.vertical_tolerance_m
            and (
                not self.enable_yaw_control
                or abs(float(self.latest_filtered_yaw_error)) <= self.yaw_tolerance_rad
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

    def actual_gate_reason(self, now, pose_fresh):
        if not self.state_is_fresh(now) or not self.state.connected:
            return 'WAIT_FCU'
        if not self.state.armed:
            return 'WAIT_ARM'
        if self.state.mode != 'OFFBOARD':
            return 'WAIT_OFFBOARD'
        if not pose_fresh:
            return 'WAIT_POSE'
        return None

    def control_loop(self, _event):
        with self.lock:
            now = rospy.Time.now()
            pose_fresh = self.observation_is_fresh(now)
            if pose_fresh:
                body_velocity = tuple(self.latest_body_velocity)
                yaw_rate = self.latest_yaw_rate
                nav_state = self.update_alignment_state(now)
            else:
                if self.latest_position_error is not None:
                    self.invalidate_observation('Pose 超时')
                body_velocity = (0.0, 0.0, 0.0)
                yaw_rate = 0.0
                nav_state = 'WAIT_POSE'

            gate_reason = None
            actual_allowed = False
            if self.enable_setpoint_output:
                gate_reason = self.actual_gate_reason(now, pose_fresh)
                actual_allowed = gate_reason is None

            self.publish_commands(now, body_velocity, yaw_rate, actual_allowed)

            if pose_fresh:
                preview_text = (
                    '%s | yaw_control=%s | error_body=(%.3f, %.3f, %.3f)m '
                    'yaw_raw=%.2fdeg yaw_lpf=%.2fdeg | '
                    'cmd_flu=(%.3f, %.3f, %.3f, %.3f)'
                    % (
                        nav_state,
                        'ON' if self.enable_yaw_control else 'OFF',
                        self.latest_position_error[0],
                        self.latest_position_error[1],
                        self.latest_position_error[2],
                        math.degrees(self.latest_yaw_error),
                        math.degrees(self.latest_filtered_yaw_error),
                        body_velocity[0],
                        body_velocity[1],
                        body_velocity[2],
                        yaw_rate,
                    )
                )
            else:
                preview_text = '%s | yaw_control=%s | %s | cmd=zero' % (
                    nav_state,
                    'ON' if self.enable_yaw_control else 'OFF',
                    self.last_observation_issue or 'Pose 超时',
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

    def publish_commands(self, stamp, body_velocity, yaw_rate, actual_allowed):
        if not self.enable_yaw_control:
            yaw_rate = 0.0

        command = TwistStamped()
        command.header.stamp = stamp
        command.header.frame_id = self.base_frame
        command.twist.linear.x = float(body_velocity[0])
        command.twist.linear.y = float(body_velocity[1])
        command.twist.linear.z = float(body_velocity[2])
        command.twist.angular.z = float(yaw_rate)
        self.command_pub.publish(command)

        preview = self.make_setpoint(stamp, body_velocity, yaw_rate)
        self.preview_pub.publish(preview)
        if self.setpoint_pub is not None:
            actual = preview if actual_allowed else self.make_setpoint(
                stamp, (0.0, 0.0, 0.0), 0.0
            )
            self.setpoint_pub.publish(actual)

    def status_loop(self, _event):
        with self.lock:
            text = self.last_status
        self.status_pub.publish(String(data=text))
        rospy.loginfo(text)


def main():
    rospy.init_node('front_marker_control_node')
    FrontMarkerControlNode()
    rospy.spin()
