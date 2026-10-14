"""Pose-driven body-velocity controller with gated MAVROS output."""

from collections import deque
import math
from statistics import median
import threading

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Float64, String
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped conversions
import tf2_ros

from front_marker_control.control import (
    BodyVelocityController,
    Vector3SlewRateLimiter,
    wrap_angle,
)
from front_marker_control.geometry import (
    finite_vector3,
    marker_heading_body,
    normalize_quaternion,
    position_error_body,
    position_hold_target,
    rotate_vector_by_quaternion,
    rotate_vector_by_yaw,
    yaw_from_quaternion,
)
from front_marker_control.refinement import DEFAULTS, FinalPoseRefiner, validate_parameters
from front_marker_control.localization import LocalPoseBuffer
from front_marker_control.mavros_interface import (
    body_flu_setpoint_components,
    position_only_type_mask,
    velocity_only_type_mask,
    velocity_absolute_yaw_type_mask,
)

# yaw 来源：fixed_face 用定位锁货架朝向；marker 用 tag yaw（原流程）；
# off 不控 yaw（enable_yaw_control=false）。
YAW_MODES = ('fixed_face', 'marker')


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
        self.last_filter_stamp = None
        self.last_accepted_source_stamp = None
        self.aligned_start_time = None
        self.latest_body_velocity = (0.0, 0.0, 0.0)
        self.latest_yaw_rate = 0.0
        self.last_control_time = None
        self.last_observation_issue = '尚未收到 Marker Pose'
        self.last_status = 'PREVIEW_ONLY | WAIT_POSE'
        self.last_motion_state = None
        self.last_control_mode = None
        self.last_mode_reason = None

        # 位置锁定状态：ALIGN -> POS_HOLD -> RETREAT。
        self.control_mode = 'ALIGN'
        self.mode_reason = '初始'
        # (p_target ENU, yaw_target ENU rad, lock marker stamp)
        self.position_target = None
        self.pos_exit_start_time = None
        self.local_pose_buffer = LocalPoseBuffer()
        self.latest_odom_receive_time = None
        self.latest_odom_source_stamp = None
        self.last_odom_issue = '尚未收到 Odometry'
        self.last_localization_issue = ''
        self.latest_local_pose = None
        self.latest_local_speed = float('inf')
        # fixed_face：本次 OFFBOARD 锁定的货架朝向与斜坡中的 yaw 设定（local ENU rad）。
        self.face_yaw = None
        self.face_yaw_command = None
        self.face_issue = ''
        # 机体速度（Odometry twist 模长），仅用于 fixed_face 锁定前的静止判定。
        self.latest_local_velocity_norm = float('inf')
        # 最近的 Marker 观测（源时间 s, base_link 下 Marker 位置），锁定时取对准窗口中位数。
        self.recent_markers = deque(maxlen=64)
        self.aligned_start_stamp = None
        self.refiner = None
        if self.enable_position_hold and self.enable_yaw_control:
            self.refiner = FinalPoseRefiner(
                self.refinement_parameters, self.target_xyz_body,
                (self.distance_tolerance_m, self.lateral_tolerance_m, self.vertical_tolerance_m),
                self.odom_interp_gap_max_sec, self.pose_timeout_sec,
            )

        self.body_controller = BodyVelocityController(
            kp_x=self.kp_x,
            kp_y=self.kp_y,
            kp_z=self.kp_z,
            kp_yaw=self.kp_yaw,
            vx_limit=self.vx_limit,
            vyz_limit=self.vyz_limit,
            yaw_rate_limit=self.yaw_rate_limit,
            yaw_error_deadband=self.yaw_error_deadband,
            yaw_lpf_tau=self.yaw_lpf_tau,
            enable_yaw_control=self.enable_yaw_control,
            velocity_deadband_off_x=self.velocity_deadband_off_x,
            velocity_deadband_on_x=self.velocity_deadband_on_x,
            velocity_deadband_off_y=self.velocity_deadband_off_y,
            velocity_deadband_on_y=self.velocity_deadband_on_y,
            velocity_deadband_off_z=self.velocity_deadband_off_z,
            velocity_deadband_on_z=self.velocity_deadband_on_z,
            position_lpf_tau_x=self.position_lpf_tau_x,
            position_lpf_tau_y=self.position_lpf_tau_y,
            position_lpf_tau_z=self.position_lpf_tau_z,
        )
        self.velocity_limiter = Vector3SlewRateLimiter()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.position_error_pub = rospy.Publisher(
            self.position_error_topic, Vector3Stamped, queue_size=10
        )
        self.yaw_error_pub = rospy.Publisher(
            self.yaw_error_topic, Float64, queue_size=10
        )
        self.fixed_yaw_pub = rospy.Publisher(
            self.fixed_yaw_target_topic, Float64, queue_size=10, latch=True
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

        self.control_mode_pub = rospy.Publisher(
            self.control_mode_topic, String, queue_size=10, latch=True
        )
        self.last_published_control_mode = None

        self.setpoint_pub = None
        if self.enable_setpoint_output:
            self.setpoint_pub = rospy.Publisher(
                self.setpoint_topic, PositionTarget, queue_size=10
            )
            rospy.logwarn(
                '真实 MAVROS setpoint 已启用；非零输出要求 connected + armed + OFFBOARD，'
                '视觉失效后仅允许受控刹车'
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
        if self.enable_position_hold:
            rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self.odom_callback,
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
            'target=(%.3f, %.3f, %.3f)m | '
            'position_gains=(kp_x=%.3f, kp_y=%.3f, kp_z=%.3f) | '
            'yaw_gain=%.3f | yaw_mode=%s | '
            'setpoint_rate=%.1fHz | max_accel=%.3fm/s^2 | '
            'loss_decel=%.3fm/s^2 | pose_timeout=%.3fs | '
            'max_pose_latency=%.3fs | filter_dt_max=%.3fs | setpoint_output=%s | '
            'position_hold=%s | odom=%s | odom_timeout=%.3fs | '
            'pos_exit=(%.2f, %.2f, %.2f)m %.1fdeg %.2fs | '
            'near_wall=%.2fm->%.2fm retreat_vx=%.2fm/s | control_mode=%s',
            self.pose_topic,
            self.base_frame,
            self.target_xyz_body[0],
            self.target_xyz_body[1],
            self.target_xyz_body[2],
            self.kp_x,
            self.kp_y,
            self.kp_z,
            self.kp_yaw,
            self.yaw_mode,
            self.setpoint_publish_rate_hz,
            self.max_accel,
            self.loss_decel,
            self.pose_timeout_sec,
            self.max_pose_latency_sec,
            self.filter_dt_max_sec,
            self.enable_setpoint_output,
            self.enable_position_hold,
            self.odom_topic,
            self.odom_timeout_sec,
            self.pos_exit_distance_tolerance_m,
            self.pos_exit_lateral_tolerance_m,
            self.pos_exit_vertical_tolerance_m,
            self.pos_exit_yaw_tolerance_deg,
            self.pos_exit_hold_sec,
            self.min_marker_distance_m,
            self.retreat_recover_distance_m,
            self.retreat_velocity_x,
            self.control_mode_topic,
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
        self.control_mode_topic = str(
            get('~control_mode_topic', '/front_marker_control/control_mode')
        )
        self.fixed_yaw_target_topic = str(get(
            '~fixed_yaw_target_topic', '/front_marker_control/fixed_yaw_target'))
        self.refinement_parameters = {name: float(get('~' + name, value))
                                      for name, value in DEFAULTS.items()}
        validate_parameters(self.refinement_parameters)
        self.base_frame = str(get('~base_frame', 'base_link'))
        if not self.base_frame:
            raise ValueError('base_frame must not be empty')
        self.target_xyz_body = finite_vector3(
            get('~target_xyz_body', [0.8, 0.0, 0.0]),
            'target_xyz_body',
        )

        numeric_defaults = {
            'kp_x': 0.50,
            'kp_y': 0.60,
            'kp_z': 0.60,
            'kp_yaw': 0.40,
            'position_lpf_tau_x': 0.05,
            'position_lpf_tau_y': 0.10,
            'position_lpf_tau_z': 0.10,
            'yaw_lpf_tau': 0.20,
            'vyz_limit': 0.80,
            'vx_limit': 0.20,
            'yaw_rate_limit': 0.80,
            'yaw_error_deadband': 0.06,
            'velocity_deadband_off_x': 0.010,
            'velocity_deadband_on_x': 0.015,
            'velocity_deadband_off_y': 0.004,
            'velocity_deadband_on_y': 0.007,
            'velocity_deadband_off_z': 0.003,
            'velocity_deadband_on_z': 0.006,
            'setpoint_publish_rate_hz': 20.0,
            'max_accel': 0.50,
            'loss_decel': 1.00,
            'pose_timeout_sec': 0.1,
            'max_pose_latency_sec': 0.5,
            'filter_dt_max_sec': 0.5,
            'state_timeout_sec': 2.0,
            'future_stamp_tolerance_sec': 0.1,
            'tf_lookup_timeout_sec': 0.05,
            'lateral_tolerance_m': 0.08,
            'vertical_tolerance_m': 0.08,
            'distance_tolerance_m': 0.12,
            'yaw_tolerance_deg': 6.0,
            'aligned_hold_sec': 1.0,
            'odom_timeout_sec': 0.20,
            'odom_interp_gap_max_sec': 0.10,
            'odom_source_max_age_sec': 0.5,
            'pos_exit_distance_tolerance_m': 0.18,
            'pos_exit_lateral_tolerance_m': 0.12,
            'pos_exit_vertical_tolerance_m': 0.12,
            'pos_exit_yaw_tolerance_deg': 9.0,
            'pos_exit_hold_sec': 1.0,
            'min_marker_distance_m': 0.50,
            'retreat_recover_distance_m': 0.70,
            'retreat_velocity_x': -0.10,
            'face_yaw_max_offset_deg': 30.0,
            'face_yaw_rate_deg_s': 10.0,
            'lock_max_speed_mps': 0.05,
            'lock_min_samples': 5.0,
        }
        for name, default in numeric_defaults.items():
            setattr(self, name, float(get('~' + name, default)))

        self.enable_setpoint_output = bool(get('~enable_setpoint_output', False))
        self.enable_position_hold = bool(get('~enable_position_hold', False))
        # enable_yaw_control 仍是总开关；开启时由 yaw_mode 选择 yaw 来源。
        # 之后 self.enable_yaw_control 只表示“tag yaw 进入控制环”（marker 模式），
        # fixed_face 与 off 共用其关闭时的路径（不发 yaw 角速度、不建 3 s 采样器）。
        if bool(get('~enable_yaw_control', True)):
            self.yaw_mode = str(get('~yaw_mode', 'fixed_face'))
            if self.yaw_mode not in YAW_MODES:
                raise ValueError(
                    'yaw_mode must be one of %s, got %r' % (YAW_MODES, self.yaw_mode)
                )
        else:
            self.yaw_mode = 'off'
        self.enable_yaw_control = self.yaw_mode == 'marker'
        if self.yaw_mode == 'fixed_face' and not self.enable_position_hold:
            raise ValueError(
                'yaw_mode=fixed_face 需要 enable_position_hold=true（用定位锁 yaw 与悬停点）'
            )
        self.face_yaws = tuple(
            wrap_angle(math.radians(float(value)))
            for value in get('~face_yaws_deg', [0.0, 180.0])
        )
        if not self.face_yaws or not all(math.isfinite(v) for v in self.face_yaws):
            raise ValueError('face_yaws_deg must be a non-empty list of finite angles')
        self.face_yaw_max_offset_rad = math.radians(self.face_yaw_max_offset_deg)
        self.face_yaw_rate_rad_s = math.radians(self.face_yaw_rate_deg_s)
        self.lock_min_samples = int(self.lock_min_samples)
        self.yaw_tolerance_rad = math.radians(self.yaw_tolerance_deg)
        self.pos_exit_yaw_tolerance_rad = math.radians(
            self.pos_exit_yaw_tolerance_deg
        )
        self.odom_topic = str(get('~odom_topic', '/mavros/local_position/odom'))

        for name in numeric_defaults:
            if not math.isfinite(getattr(self, name)):
                raise ValueError('%s must be finite' % name)
        if self.setpoint_publish_rate_hz <= 0.0:
            raise ValueError('setpoint_publish_rate_hz must be positive')
        if self.max_accel <= 0.0 or self.loss_decel <= 0.0:
            raise ValueError('max_accel and loss_decel must be positive')
        if (
            self.pose_timeout_sec <= 0.0
            or self.max_pose_latency_sec <= 0.0
            or self.filter_dt_max_sec <= 0.0
            or self.state_timeout_sec <= 0.0
        ):
            raise ValueError(
                'pose_timeout_sec, max_pose_latency_sec, filter_dt_max_sec and '
                'state_timeout_sec must be positive'
            )
        if (
            self.odom_timeout_sec <= 0.0
            or self.odom_interp_gap_max_sec <= 0.0
            or self.odom_source_max_age_sec <= 0.0
            or self.min_marker_distance_m <= 0.0
            or self.retreat_recover_distance_m <= 0.0
        ):
            raise ValueError(
                'odom_timeout_sec, odom_interp_gap_max_sec, '
                'odom_source_max_age_sec, min_marker_distance_m and '
                'retreat_recover_distance_m must be positive'
            )
        if self.retreat_recover_distance_m <= self.min_marker_distance_m:
            raise ValueError(
                'retreat_recover_distance_m must exceed min_marker_distance_m'
            )
        if self.target_xyz_body[0] <= self.min_marker_distance_m:
            raise ValueError(
                'target_xyz_body[0] must exceed min_marker_distance_m, '
                'otherwise the aligned lock is immediately broken by the '
                'near-wall retreat and POS_HOLD is unreachable'
            )
        if self.retreat_velocity_x >= 0.0:
            raise ValueError('retreat_velocity_x must be negative')
        if (
            self.pos_exit_distance_tolerance_m < self.distance_tolerance_m
            or self.pos_exit_lateral_tolerance_m < self.lateral_tolerance_m
            or self.pos_exit_vertical_tolerance_m < self.vertical_tolerance_m
            or self.pos_exit_yaw_tolerance_deg < self.yaw_tolerance_deg
        ):
            raise ValueError(
                'pos exit tolerances must not be below the entry tolerances'
            )
        if self.future_stamp_tolerance_sec < 0.0 or self.tf_lookup_timeout_sec < 0.0:
            raise ValueError('timestamp and TF tolerances must be non-negative')
        if (
            self.face_yaw_max_offset_deg <= 0.0
            or self.face_yaw_rate_deg_s <= 0.0
            or self.lock_max_speed_mps <= 0.0
            or self.lock_min_samples < 1
        ):
            raise ValueError(
                'face_yaw_max_offset_deg, face_yaw_rate_deg_s, lock_max_speed_mps '
                'and lock_min_samples must be positive'
            )
        if any(value < 0.0 for value in (
                self.vyz_limit,
                self.vx_limit,
                self.yaw_rate_limit,
                self.yaw_error_deadband,
                self.velocity_deadband_off_x,
                self.velocity_deadband_on_x,
                self.velocity_deadband_off_y,
                self.velocity_deadband_on_y,
                self.velocity_deadband_off_z,
                self.velocity_deadband_on_z,
                self.position_lpf_tau_x,
                self.position_lpf_tau_y,
                self.position_lpf_tau_z,
                self.yaw_lpf_tau,
                self.lateral_tolerance_m,
                self.vertical_tolerance_m,
                self.distance_tolerance_m,
                self.yaw_tolerance_deg,
                self.aligned_hold_sec,
                self.pos_exit_distance_tolerance_m,
                self.pos_exit_lateral_tolerance_m,
                self.pos_exit_vertical_tolerance_m,
                self.pos_exit_yaw_tolerance_deg,
                self.pos_exit_hold_sec,
        )):
            raise ValueError('limits, deadbands and tolerances must be non-negative')
        for axis in ('x', 'y', 'z'):
            if getattr(self, 'velocity_deadband_off_' + axis) > getattr(
                self, 'velocity_deadband_on_' + axis
            ):
                raise ValueError(
                    'velocity_deadband_off_%s must not exceed '
                    'velocity_deadband_on_%s' % (axis, axis)
                )

    def state_callback(self, message):
        with self.lock:
            self.state = message
            now = rospy.Time.now()
            interrupted = (self.latest_state_receive_time is not None
                           and (now-self.latest_state_receive_time).to_sec() > self.state_timeout_sec)
            self.latest_state_receive_time = now
            gated = self.enable_setpoint_output and (
                interrupted or not message.connected or not message.armed or message.mode != 'OFFBOARD')
            if self.refiner is not None and gated:
                self.enter_align('FCU gate interrupted')
                self.velocity_limiter.reset()
            if self.yaw_mode == 'fixed_face' and gated:
                # 退出 OFFBOARD 即释放货架朝向，下次切入按当时航向重新选择。
                self.release_face_yaw()

    def _localization_rejected(self, issue):
        """定位数据被拒绝：立即把定位置为不可用，并原子清除位置目标。

        清除不能推迟到控制循环——若下一次有效回调先于控制循环到达，
        循环会看到“定位新鲜”而继续使用旧的 position_target。RETREAT 是
        纯视觉近墙保护，不依赖定位，因此不被这里打断。
        """
        self.latest_odom_receive_time = None
        self.last_odom_issue = issue
        self.latest_local_pose = None
        self.latest_local_speed = float('inf')
        self.latest_local_velocity_norm = float('inf')
        self.local_pose_buffer.clear()
        if self.yaw_mode == 'fixed_face':
            # 定位异常（含坐标系变化）后 yaw 参考可能已变，恢复后重新选择货架朝向。
            self.release_face_yaw()
        if self.refiner is not None:
            self.refiner.reset(issue)
        self.aligned_start_time = None
        if self.control_mode in ('YAW_SAMPLING', 'POSE_ALIGNING', 'POS_HOLD'):
            self.enter_align('定位异常 %s' % issue)
        elif self.position_target is not None:
            self.clear_position_target()

    def odom_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            # 源时间与接收时间分别校验：源时间戳判定数据本身是否新鲜/过期，
            # 接收时间判定链路是否仍在下发；两者任一异常即拒绝该帧。
            source_stamp = message.header.stamp
            if source_stamp == rospy.Time():
                self._localization_rejected('拒绝零时间戳 Odometry')
                return
            source_age = now.to_sec() - source_stamp.to_sec()
            if not math.isfinite(source_age):
                self._localization_rejected('拒绝非有限定位源时间戳')
                return
            if source_age > self.odom_source_max_age_sec:
                self._localization_rejected(
                    '拒绝过期定位源 %.1fs' % source_age
                )
                return
            if source_age < -self.future_stamp_tolerance_sec:
                self._localization_rejected(
                    '拒绝超前定位源 %.1fs' % (-source_age)
                )
                return
            receive_stamp = now.to_sec()
            if not math.isfinite(receive_stamp):
                self._localization_rejected('拒绝非有限接收时间')
                return
            # Pair on source timestamps; Marker observations wait for a bracketing odom.
            reason = self.local_pose_buffer.accept(
                source_stamp.to_sec(),
                message.header.frame_id,
                (
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    message.pose.pose.position.z,
                ),
                (
                    message.pose.pose.orientation.x,
                    message.pose.pose.orientation.y,
                    message.pose.pose.orientation.z,
                    message.pose.pose.orientation.w,
                ),
            )
            if reason is not None:
                # 非法值或坐标系变化已在 accept 内清除旧定位缓存。
                self._localization_rejected(reason)
                return
            if (self.refiner is not None and self.latest_odom_receive_time is not None
                    and (now-self.latest_odom_receive_time).to_sec() > self.odom_timeout_sec):
                # Recovery arriving before the timer must still invalidate the old epoch.
                self.refiner.reset('Odometry stream interrupted')
                self.position_target = None
                self.aligned_start_time = None
                self.pos_exit_start_time = None
                if self.control_mode != 'RETREAT':
                    self.control_mode = 'ALIGN'
            local_pose = self.local_pose_buffer.lookup(source_stamp.to_sec(), self.odom_interp_gap_max_sec)
            previous_pose = getattr(self, 'latest_local_pose', None)
            self.latest_local_speed = float('inf')
            if previous_pose is not None and local_pose.stamp > previous_pose.stamp:
                interval = local_pose.stamp - previous_pose.stamp
                self.latest_local_speed = self.velocity_norm(tuple(
                    (b-a)/interval for a, b in zip(previous_pose.position, local_pose.position)))
            # 采样期速度仅作状态展示，不再触发任何重置；
            # POSE_ALIGNING 的过渡完成判定只看轨迹自身，不设速度/位姿门。
            twist = message.twist.twist.linear
            self.latest_local_velocity_norm = self.velocity_norm((twist.x, twist.y, twist.z))
            self.latest_local_pose = local_pose
            self.latest_odom_receive_time = now
            self.latest_odom_source_stamp = source_stamp
            self.last_odom_issue = ''
            self.last_localization_issue = ''

    def localization_unfresh_reason(self, now):
        """Return why position-hold localization is unusable, or '' when fresh.

        The odom *source* timestamp is validated at accept time; this only judges
        the receive-side freshness of the last accepted frame.
        """
        if self.latest_odom_receive_time is None:
            return '定位尚未收到 Odometry'
        receive_age = (now - self.latest_odom_receive_time).to_sec()
        if not math.isfinite(receive_age) or receive_age < 0.0:
            return '定位接收时间异常'
        if receive_age > self.odom_timeout_sec:
            return '定位接收超时 %.1fs' % receive_age
        return ''

    def localization_is_fresh(self, now):
        return not self.localization_unfresh_reason(now)

    def _report_localization_lost(self, now):
        """Expose the localization failure reason and log it throttled."""
        reason = self.localization_unfresh_reason(now)
        if not reason:
            return
        # 具体的拒绝原因（过期源/非法值/坐标系变化）优先于笼统的“尚未收到”。
        issue = self.last_odom_issue or reason
        self.last_localization_issue = issue
        rospy.logwarn_throttle(
            5.0, '位置锁定暂停：定位不足（%s），等待定位恢复', issue,
        )

    def clear_position_target(self):
        if self.refiner is not None:
            self.refiner.reset('position target cleared')
        self.position_target = None
        self.pos_exit_start_time = None

    def enter_align(self, reason):
        self.clear_position_target()
        self.aligned_start_time = None
        self.control_mode = 'ALIGN'
        self.mode_reason = reason

    def enter_retreat(self, reason):
        self.clear_position_target()
        self.aligned_start_time = None
        self.control_mode = 'RETREAT'
        self.mode_reason = reason
        # 近墙后退必须丢弃限速器里残留的历史（可能带着朝墙的正向及横向/垂向
        # 分量），否则后退首周期会继续输出旧的正向速度。清空后由本周期指令
        # 生成从零按 max_accel 起步后退。
        self.velocity_limiter.reset()

    def release_face_yaw(self):
        self.face_yaw = None
        self.face_yaw_command = None

    def nearest_face_yaw(self, yaw):
        return min(self.face_yaws, key=lambda face: abs(wrap_angle(face - yaw)))

    def update_face_yaw(self, dt):
        """Latch the nearest shelf heading once per OFFBOARD and ramp the yaw setpoint.

        Returns True on the tick that latched a new heading. The caller must
        hold ``self.lock`` and guarantee a fresh ``latest_local_pose``.
        """
        current = yaw_from_quaternion(self.latest_local_pose.quaternion)
        latched = False
        if self.face_yaw is None:
            face = self.nearest_face_yaw(current)
            offset = wrap_angle(face - current)
            if abs(offset) > self.face_yaw_max_offset_rad:
                self.face_issue = '航向偏离货架朝向 %.1fdeg（上限 %.1fdeg），不进场' % (
                    math.degrees(offset), self.face_yaw_max_offset_deg)
                return False
            self.face_yaw = face
            self.face_yaw_command = current
            self.face_issue = ''
            latched = True
        # 从当前航向按限定角速度转到货架朝向，避免切入瞬间的 yaw 阶跃。
        step = self.face_yaw_rate_rad_s * dt
        delta = wrap_angle(self.face_yaw - self.face_yaw_command)
        if abs(delta) <= step + 1e-9:
            self.face_yaw_command = self.face_yaw
        else:
            self.face_yaw_command = wrap_angle(
                self.face_yaw_command + math.copysign(step, delta))
        return latched

    def face_velocity_target(self, body_velocity):
        """Body FLU velocity as a local ENU velocity with the ramped absolute yaw."""
        return (
            rotate_vector_by_quaternion(self.latest_local_pose.quaternion, body_velocity),
            self.face_yaw_command,
        )

    def face_status(self):
        if self.face_yaw is None:
            return 'face=未锁定 %s' % self.face_issue if self.face_issue else 'face=未锁定'
        return 'face=%.1fdeg yaw_sp=%.1fdeg' % (
            math.degrees(self.face_yaw), math.degrees(self.face_yaw_command))

    def yaw_control_label(self):
        if self.enable_yaw_control:
            return 'ON'
        return 'FACE' if self.yaw_mode == 'fixed_face' else 'OFF'

    def face_yaw_settled(self):
        """Yaw setpoint reached the shelf heading and the vehicle follows it."""
        if self.face_yaw is None or self.face_yaw_command != self.face_yaw:
            return False
        if self.latest_local_pose is None:
            return False
        current = yaw_from_quaternion(self.latest_local_pose.quaternion)
        return abs(wrap_angle(current - self.face_yaw)) <= self.yaw_tolerance_rad

    def try_lock_face_target(self):
        """Lock the hold point from the median Marker position of the aligned window.

        Returns ``(p_target, face_yaw, last_marker_stamp)`` or None. The hold
        yaw is the latched shelf heading; the tag yaw is not used.
        """
        markers = []
        last_stamp = None
        for stamp, marker_body in self.recent_markers:
            if stamp < self.aligned_start_stamp:
                continue
            local_pose = self.local_pose_buffer.lookup(stamp, self.odom_interp_gap_max_sec)
            if local_pose is None:
                continue
            rotated = rotate_vector_by_quaternion(local_pose.quaternion, marker_body)
            markers.append(tuple(a + b for a, b in zip(local_pose.position, rotated)))
            last_stamp = stamp
        if len(markers) < self.lock_min_samples:
            self.last_localization_issue = '锁定样本不足 %d/%d' % (
                len(markers), self.lock_min_samples)
            return None
        p_marker = tuple(median(marker[axis] for marker in markers) for axis in range(3))
        offset = rotate_vector_by_yaw(self.target_xyz_body, self.face_yaw)
        p_target = tuple(a - b for a, b in zip(p_marker, offset))
        self.last_localization_issue = ''
        return (p_target, self.face_yaw, last_stamp)

    def position_exit_exceeded(self):
        """True when the live vision error exceeds the POS_HOLD exit band.

        仅作用于未启用 yaw 精对准的旧路径（refiner=None）；启用精对准时
        目标固定，不再按视觉误差退出，退出只来自近墙/VIO/FCU/显式重置。
        """
        if self.refiner is not None:
            return False
        error = self.latest_position_error
        if (
            abs(float(error[0])) > self.pos_exit_distance_tolerance_m
            or abs(float(error[1])) > self.pos_exit_lateral_tolerance_m
            or abs(float(error[2])) > self.pos_exit_vertical_tolerance_m
        ):
            return True
        return (
            self.refiner is None and self.enable_yaw_control
            and abs(float(self.latest_yaw_error)) > self.pos_exit_yaw_tolerance_rad
        )

    def update_control_mode(self, now, pose_fresh, marker_x, near_wall):
        """Advance the ALIGN/POS_HOLD/RETREAT control-mode state machine."""
        if self.control_mode == 'RETREAT':
            if pose_fresh and marker_x >= self.retreat_recover_distance_m:
                self.enter_align('后退恢复 marker_x=%.2fm' % marker_x)
            return
        if pose_fresh and near_wall:
            # 近墙保护优先于 ALIGN 与 POS_HOLD。
            self.enter_retreat('近墙保护 marker_x=%.2fm' % marker_x)
            return
        if self.control_mode == 'POS_HOLD':
            if self.position_target is None:
                self.enter_align('位置目标缺失')
            elif pose_fresh and self.position_exit_exceeded():
                if self.pos_exit_start_time is None:
                    self.pos_exit_start_time = now
                elif (
                    (now - self.pos_exit_start_time).to_sec()
                    >= self.pos_exit_hold_sec
                ):
                    self.enter_align('视觉误差超限退出')
            else:
                # 丢标（pose_fresh=False）期间不累计退出计时：恢复首帧仍需
                # 重新连续超限满 hold_sec 才会退出，避免中断后立刻误退出。
                self.pos_exit_start_time = None

    def try_lock_position_target(self):
        """Compute and lock the fixed XYZ/yaw target, or return None."""
        if self.yaw_mode == 'fixed_face':
            return self.try_lock_face_target()
        # Legacy yaw-disabled lock also uses source-time interpolation.
        marker_stamp = self.latest_source_stamp.to_sec()
        local_pose = self.local_pose_buffer.lookup(
            marker_stamp, self.odom_interp_gap_max_sec
        )
        if local_pose is None:
            self.last_localization_issue = '定位无法与 Marker 时间对齐'
            return None
        marker_position_body = tuple(
            self.latest_position_error[index] + self.target_xyz_body[index]
            for index in range(3)
        )
        try:
            p_target, yaw_target = position_hold_target(
                local_pose.position,
                local_pose.quaternion,
                marker_position_body,
                self.latest_yaw_error,
                self.target_xyz_body,
                apply_yaw_correction=self.enable_yaw_control,
            )
        except ValueError as exception:
            self.last_localization_issue = '锁定目标计算失败: %s' % exception
            return None
        self.last_localization_issue = ''
        return (p_target, yaw_target, marker_stamp)

    def invalidate_observation(self, reason):
        # 视觉失效不再重置采样器：固定窗口靠 VIO/FCU/近墙/显式重置打断。
        self.latest_position_error = None
        self.latest_yaw_error = 0.0
        self.latest_filtered_yaw_error = 0.0
        self.latest_source_stamp = None
        self.latest_receive_time = None
        self.last_observation_issue = reason
        self.reset_tracking_target()

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

    def observation_stamp_issue(self, stamp, now):
        """Return a rejection reason for a source stamp.

        The caller must hold ``self.lock`` because the accepted source stamp is
        shared with other Pose callback threads.
        """
        if stamp == rospy.Time():
            return '拒绝零时间戳 Pose'
        if self.last_accepted_source_stamp is not None:
            if stamp == self.last_accepted_source_stamp:
                return '拒绝重复时间戳 Pose'
            if stamp < self.last_accepted_source_stamp:
                return '拒绝倒退时间戳 Pose'

        source_age = (now - stamp).to_sec()
        if source_age > self.max_pose_latency_sec:
            return '收到时已过期的 Pose，source_age=%.3fs' % source_age
        if source_age < -self.future_stamp_tolerance_sec:
            return '拒绝未来时间戳 Pose，source_age=%.3fs' % source_age
        return None

    def pose_callback(self, message):
        now = rospy.Time.now()

        # Reject obviously unusable samples before doing a potentially blocking
        # TF lookup. This first check is repeated at commit time because another
        # subscriber thread may accept a newer sample while TF is running.
        with self.lock:
            stamp_issue = self.observation_stamp_issue(message.header.stamp, now)
            if stamp_issue is not None:
                self.invalidate_observation(stamp_issue)
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
            rejection_reason = '拒绝非法 Pose: %s' % exception
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exception:
            rejection_reason = 'WAIT_TF: %s' % exception
        else:
            rejection_reason = None

        publish_snapshot = None
        with self.lock:
            # Preserve timestamp rejection precedence and ensure an older Pose
            # cannot overwrite state committed while its TF lookup was running.
            stamp_issue = self.observation_stamp_issue(message.header.stamp, now)
            if stamp_issue is not None:
                self.invalidate_observation(stamp_issue)
                return
            if rejection_reason is not None:
                self.invalidate_observation(rejection_reason)
                return

            if self.last_filter_stamp is None:
                self.reset_controller_state()
                dt = 1.0 / self.setpoint_publish_rate_hz
            else:
                dt = (message.header.stamp - self.last_filter_stamp).to_sec()
                if not math.isfinite(dt) or dt <= 0.0 or dt > self.filter_dt_max_sec:
                    self.reset_controller_state()
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
            self.last_filter_stamp = message.header.stamp
            self.last_accepted_source_stamp = message.header.stamp
            self.last_observation_issue = ''
            if self.refiner is not None:
                # 固定 3 秒窗口：断帧/超速/XYZ 偏离均不重置采样，
                # Marker 观测照常入队等待源时间配对。
                self.refiner.marker(message.header.stamp.to_sec(), now.to_sec(),
                                    tuple(error[i]+self.target_xyz_body[i] for i in range(3)), yaw_error)
            if self.yaw_mode == 'fixed_face':
                self.recent_markers.append((
                    message.header.stamp.to_sec(),
                    tuple(error[i] + self.target_xyz_body[i] for i in range(3)),
                ))
            publish_snapshot = (
                message.header.stamp,
                tuple(float(value) for value in error),
                float(yaw_error),
            )

        self.publish_errors(*publish_snapshot)

    def publish_errors(self, stamp, error, yaw_error):
        message = Vector3Stamped()
        message.header.stamp = stamp
        message.header.frame_id = self.base_frame
        message.vector.x = float(error[0])
        message.vector.y = float(error[1])
        message.vector.z = float(error[2])
        self._publish_if_running(self.position_error_pub, message)
        self._publish_if_running(
            self.yaw_error_pub, Float64(data=float(yaw_error))
        )

    def observation_is_fresh(self, now):
        if (
            self.latest_position_error is None
            or self.latest_source_stamp is None
            or self.latest_receive_time is None
        ):
            return False
        receive_age = (now - self.latest_receive_time).to_sec()
        return 0.0 <= receive_age <= self.pose_timeout_sec

    def state_is_fresh(self, now):
        if self.latest_state_receive_time is None:
            return False
        age = (now - self.latest_state_receive_time).to_sec()
        return 0.0 <= age <= self.state_timeout_sec

    def reset_controller_state(self):
        self.body_controller.reset()

    def reset_tracking_target(self):
        self.reset_controller_state()
        self.last_filter_stamp = None
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
            # fixed_face：yaw 只看定位，设定转到货架朝向且机体跟上后才计时。
            and (self.yaw_mode != 'fixed_face' or self.face_yaw_settled())
        )
        if not aligned:
            self.aligned_start_time = None
            return 'ALIGN'
        if self.aligned_start_time is None:
            self.aligned_start_time = now
            self.aligned_start_stamp = self.latest_source_stamp.to_sec()
            return 'ALIGN'
        held = (now - self.aligned_start_time).to_sec()
        if held < self.aligned_hold_sec:
            return 'ALIGN'
        if (
            self.yaw_mode == 'fixed_face'
            and self.latest_local_velocity_norm > self.lock_max_speed_mps
        ):
            # 到位不等于停稳：对准已满保持时间，等机体速度降下来再锁定。
            return 'WAIT_STILL'
        return 'ALIGNED'

    def hard_gate_reason(self, now):
        if not self.state_is_fresh(now) or not self.state.connected:
            return 'WAIT_FCU'
        if not self.state.armed:
            return 'WAIT_ARM'
        if self.state.mode != 'OFFBOARD':
            return 'WAIT_OFFBOARD'
        return None

    def command_dt(self, now):
        nominal_dt = 1.0 / self.setpoint_publish_rate_hz
        if self.last_control_time is None:
            dt = nominal_dt
        else:
            dt = (now - self.last_control_time).to_sec()
            if not math.isfinite(dt) or dt <= 0.0:
                dt = nominal_dt
        self.last_control_time = now
        return min(dt, 2.0 * nominal_dt)

    @staticmethod
    def velocity_norm(body_velocity):
        return math.sqrt(sum(float(value) ** 2 for value in body_velocity))

    @staticmethod
    def _publish_if_running(publisher, message):
        """Publish unless rospy is shutting down; preserve real publish failures."""
        if rospy.is_shutdown():
            return False
        try:
            publisher.publish(message)
        except rospy.ROSException as exception:
            # Timer/subscriber callbacks can race with publisher closure at exit.
            if not rospy.is_shutdown() and 'closed topic' not in str(exception):
                raise
            return False
        return True

    def control_loop(self, _event):
        status_to_publish = None
        with self.lock:
            now = rospy.Time.now()
            dt = self.command_dt(now)
            pose_fresh = self.observation_is_fresh(now)
            localization_ok = (
                self.enable_position_hold and self.localization_is_fresh(now)
            )
            gate_reason = (
                self.hard_gate_reason(now) if self.enable_setpoint_output else None
            )

            fixed_yaw_snapshot = None
            if self.yaw_mode == 'fixed_face':
                # 货架朝向先于对齐判定更新：本周期的 settled 判定用最新 yaw 设定。
                if gate_reason is not None:
                    self.release_face_yaw()
                elif localization_ok and self.update_face_yaw(dt):
                    fixed_yaw_snapshot = self.face_yaw

            if not localization_ok and self.enable_position_hold:
                # 定位失效：清除位置目标与计时，锁定等待定位恢复。
                if self.position_target is not None:
                    self.enter_align('定位失效清除目标')
                self.aligned_start_time = None
                self.pos_exit_start_time = None
                self._report_localization_lost(now)
            elif pose_fresh and self.refiner is None:
                # 有新鲜视觉时推进对齐判定（ALIGNED 状态与视觉误差门限共用）。
                alignment_state = self.update_alignment_state(now)
                if (
                    alignment_state == 'ALIGNED'
                    and self.control_mode == 'ALIGN'
                    and self.enable_position_hold
                    and (not self.enable_setpoint_output or gate_reason is None)
                    and localization_ok
                ):
                    locked = self.try_lock_position_target()
                    if locked is not None:
                        self.position_target = locked
                        self.control_mode = 'POS_HOLD'
                        self.mode_reason = '对齐锁定'
                        self.pos_exit_start_time = None
            else:
                alignment_state = None

            # 状态切换（近墙保护 RETREAT / POS_HOLD 退出）须在本周期指令生成
            # 之前完成：避免触发当周期仍发送旧模式的指令（如朝墙的跟踪速度）。
            if gate_reason is None:
                if pose_fresh and self.enable_position_hold:
                    marker_x = (
                        self.latest_position_error[0] + self.target_xyz_body[0]
                    )
                    near_wall = float(marker_x) < self.min_marker_distance_m
                else:
                    marker_x = None
                    near_wall = False
                self.update_control_mode(now, pose_fresh, marker_x, near_wall)

            local_velocity_snapshot = None
            # fixed_face：速度指令转到 local 系并带绝对 yaw，由 PX4 锁航向。
            face_output = (
                self.yaw_mode == 'fixed_face'
                and localization_ok
                and self.face_yaw_command is not None
            )
            if self.refiner is not None and self.control_mode != 'RETREAT':
                if gate_reason is not None or not localization_ok:
                    self.enter_align(gate_reason or 'localization unavailable')
                else:
                    generation = self.refiner.generation
                    self.refiner.tick(now.to_sec(), dt, pose_fresh, self.latest_local_pose,
                                      self.latest_local_speed, self.latest_position_error,
                                      self.local_pose_buffer)
                    self.control_mode = self.refiner.mode
                    self.mode_reason = self.refiner.reason
                    self.position_target = (self.refiner.target + (self.refiner.target_stamp,)
                                            if self.refiner.target is not None else None)
                    if self.refiner.generation != generation:
                        fixed_yaw_snapshot = self.refiner.target[1]

            if gate_reason is not None:
                # 非 OFFBOARD、未解锁或 FCU 失联：清除位置目标与计时，不累计。
                self.enter_align('硬门控 %s' % gate_reason)
                self.velocity_limiter.reset()
                body_velocity = (0.0, 0.0, 0.0)
                yaw_rate = 0.0
                motion_state = gate_reason
                actual_allowed = False
                setpoint_snapshot = None
            else:
                if self.control_mode == 'RETREAT':
                    # 近墙后退只依赖视觉、不依赖定位：不因定位失效被清零（安全兜底优先）。
                    if pose_fresh:
                        # 后退：取消正向速度，横向/垂向与 yaw 角速度置零。
                        target_velocity = (self.retreat_velocity_x, 0.0, 0.0)
                        rate_limit = self.max_accel
                    else:
                        # 后退中丢标：目标速度归零，按丢标减速收零。
                        target_velocity = (0.0, 0.0, 0.0)
                        rate_limit = self.loss_decel
                    body_velocity = self.velocity_limiter.update(
                        target_velocity, rate_limit, dt
                    )
                    yaw_rate = 0.0
                    if face_output:
                        local_velocity_snapshot = self.face_velocity_target(body_velocity)
                    if pose_fresh:
                        motion_state = 'RETREAT'
                    elif self.velocity_norm(body_velocity) > 1e-9:
                        motion_state = 'BRAKING_POSE_LOSS'
                    else:
                        motion_state = 'RETREAT_WAIT_POSE'
                    actual_allowed = self.enable_setpoint_output
                    setpoint_snapshot = None
                elif not localization_ok and self.enable_position_hold:
                    # 定位失效：输出零速度（不做丢标制动推进）。
                    self.velocity_limiter.reset()
                    body_velocity = (0.0, 0.0, 0.0)
                    yaw_rate = 0.0
                    motion_state = 'WAIT_LOCAL_POSITION'
                    actual_allowed = self.enable_setpoint_output
                    setpoint_snapshot = None
                elif self.control_mode == 'YAW_SAMPLING':
                    # 采样期间视觉新鲜时继续 XYZ 速度闭环；丢标则按丢标
                    # 减速归零（不沿用旧视觉速度），采样计时照常推进。
                    if pose_fresh:
                        target_body_velocity = tuple(self.latest_body_velocity)
                        rate_limit = self.max_accel
                    else:
                        target_body_velocity = (0.0, 0.0, 0.0)
                        rate_limit = self.loss_decel
                    body_velocity = self.velocity_limiter.update(
                        target_body_velocity, rate_limit, dt)
                    local_velocity_snapshot = (
                        rotate_vector_by_quaternion(self.latest_local_pose.quaternion, body_velocity),
                        self.refiner.hold_yaw,
                    )
                    yaw_rate = 0.0
                    if pose_fresh:
                        motion_state = 'YAW_SAMPLING'
                    elif self.velocity_norm(body_velocity) > 1e-9:
                        motion_state = 'SAMPLING_BRAKING_POSE_LOSS'
                    else:
                        motion_state = 'SAMPLING_WAIT_POSE'
                    actual_allowed = self.enable_setpoint_output
                    setpoint_snapshot = None
                elif self.control_mode == 'POSE_ALIGNING':
                    # 截止周期起立即输出位置 setpoint（平滑过渡轨迹）。
                    setpoint_snapshot = self.refiner.transition.command
                    body_velocity = self.velocity_limiter.reset()
                    yaw_rate = 0.0
                    motion_state = 'POSE_ALIGNING'
                    actual_allowed = self.enable_setpoint_output
                elif self.control_mode == 'POS_HOLD':
                    # 固定 XYZ + 绝对 yaw，目标不随每帧视觉变化。
                    setpoint_snapshot = (
                        self.position_target[0],
                        self.position_target[1],
                    )
                    body_velocity = self.velocity_limiter.reset()
                    yaw_rate = 0.0
                    motion_state = 'POS_HOLD'
                    actual_allowed = self.enable_setpoint_output
                elif self.yaw_mode == 'fixed_face' and self.face_yaw is None:
                    # 航向离货架朝向太远：不进场、不转向，输出零速度等飞手处理。
                    self.velocity_limiter.reset()
                    body_velocity = (0.0, 0.0, 0.0)
                    yaw_rate = 0.0
                    motion_state = 'WAIT_FACE_YAW'
                    actual_allowed = self.enable_setpoint_output
                    setpoint_snapshot = None
                else:
                    if pose_fresh:
                        target_body_velocity = tuple(self.latest_body_velocity)
                        yaw_rate = self.latest_yaw_rate
                    else:
                        if self.latest_position_error is not None:
                            self.invalidate_observation('Pose 超时')
                        self.enter_align_if_needed('丢标重新对齐')
                        target_body_velocity = (0.0, 0.0, 0.0)
                        yaw_rate = 0.0
                    rate_limit = self.max_accel if pose_fresh else self.loss_decel
                    body_velocity = self.velocity_limiter.update(
                        target_body_velocity, rate_limit, dt
                    )
                    if face_output:
                        local_velocity_snapshot = self.face_velocity_target(body_velocity)
                    if pose_fresh:
                        motion_state = 'TRACKING'
                    elif self.velocity_norm(body_velocity) > 1e-9:
                        motion_state = 'BRAKING_POSE_LOSS'
                    else:
                        motion_state = 'WAIT_POSE'
                    actual_allowed = self.enable_setpoint_output
                    setpoint_snapshot = None

            control_mode_to_publish = None
            if self.control_mode != self.last_published_control_mode:
                # 模式话题只在变化时发布（latch）：到位自动读码等下游按跳变触发。
                self.last_published_control_mode = self.control_mode
                control_mode_to_publish = self.control_mode

            preview_text = self.build_preview_text(
                pose_fresh, body_velocity, yaw_rate, motion_state,
                setpoint_snapshot,
                alignment_state=locals().get('alignment_state'),
            )
            if self.refiner is not None:
                preview_text += ' | ' + self.refiner.status()
            if self.yaw_mode == 'fixed_face':
                preview_text += ' | ' + self.face_status()
            if not self.enable_setpoint_output:
                status_text = 'PREVIEW_ONLY | ' + preview_text
            elif gate_reason is not None:
                status_text = '%s | actual=zero | %s' % (
                    gate_reason,
                    preview_text,
                )
            elif motion_state in ('BRAKING_POSE_LOSS',
                                  'SAMPLING_BRAKING_POSE_LOSS'):
                status_text = 'SETPOINT_BRAKING | ' + preview_text
            elif setpoint_snapshot is not None:
                status_text = 'SETPOINT_POSITION | ' + preview_text
            elif motion_state in ('WAIT_LOCAL_POSITION', 'WAIT_FACE_YAW'):
                status_text = '%s | actual=zero | %s' % (motion_state, preview_text)
            elif motion_state == 'SAMPLING_WAIT_POSE':
                status_text = 'WAIT_POSE | actual=zero | ' + preview_text
            elif motion_state == 'RETREAT':
                status_text = 'SETPOINT_RETREAT | ' + preview_text
            elif motion_state == 'POS_HOLD':
                status_text = 'SETPOINT_ACTIVE | ' + preview_text
            elif pose_fresh:
                status_text = 'SETPOINT_ACTIVE | ' + preview_text
            else:
                status_text = 'WAIT_POSE | actual=zero | ' + preview_text
            if self.set_status(status_text, motion_state):
                status_to_publish = status_text
            command_snapshot = (
                now,
                tuple(float(value) for value in body_velocity),
                float(yaw_rate),
                bool(actual_allowed),
            )

        self.publish_commands(
            *command_snapshot, position_target=setpoint_snapshot,
            local_velocity_target=local_velocity_snapshot,
        )
        if fixed_yaw_snapshot is not None:
            self._publish_if_running(self.fixed_yaw_pub, Float64(data=fixed_yaw_snapshot))
        if status_to_publish is not None:
            self._publish_if_running(
                self.status_pub, String(data=status_to_publish)
            )
        if control_mode_to_publish is not None:
            self._publish_if_running(
                self.control_mode_pub, String(data=control_mode_to_publish)
            )

    def enter_align_if_needed(self, reason):
        if self.control_mode != 'ALIGN':
            self.enter_align(reason)

    def build_preview_text(
        self, pose_fresh, body_velocity, yaw_rate, motion_state,
        setpoint_snapshot, alignment_state=None,
    ):
        mode_segment = 'mode=%s/%s' % (
            self.control_mode,
            self.mode_reason,
        )
        if setpoint_snapshot is not None:
            p_target, yaw_target = setpoint_snapshot
            return (
                '%s | %s | yaw_control=%s | hold_local=(%.3f, %.3f, %.3f)m '
                'yaw=%.2fdeg | %s'
                % (
                    motion_state,
                    mode_segment,
                    self.yaw_control_label(),
                    p_target[0],
                    p_target[1],
                    p_target[2],
                    math.degrees(yaw_target),
                    self.last_localization_issue or '',
                )
            ).rstrip(' |')
        if pose_fresh:
            body_error = self.latest_position_error
            if alignment_state is not None:
                state_segment = '%s/%s' % (motion_state, alignment_state)
            else:
                state_segment = motion_state
            text = (
                '%s | %s | yaw_control=%s | error_body=(%.3f, %.3f, %.3f)m '
                'yaw_raw=%.2fdeg yaw_lpf=%.2fdeg | '
                'cmd_flu=(%.3f, %.3f, %.3f, %.3f)'
                % (
                    state_segment,
                    mode_segment,
                    self.yaw_control_label(),
                    body_error[0],
                    body_error[1],
                    body_error[2],
                    math.degrees(self.latest_yaw_error),
                    math.degrees(self.latest_filtered_yaw_error),
                    body_velocity[0],
                    body_velocity[1],
                    body_velocity[2],
                    yaw_rate,
                )
            )
            if self.last_localization_issue:
                return text + ' | ' + self.last_localization_issue
            return text
        issue = self.last_observation_issue or 'Pose 超时'
        if self.enable_position_hold and not self.localization_is_fresh_markerless():
            issue = self._compose_wait_issue()
        return (
            '%s | %s | yaw_control=%s | %s | cmd_flu=(%.3f, %.3f, %.3f, %.3f)'
            % (
                motion_state,
                mode_segment,
                self.yaw_control_label(),
                issue,
                body_velocity[0],
                body_velocity[1],
                body_velocity[2],
                yaw_rate,
            )
        )

    def localization_is_fresh_markerless(self):
        return not self.enable_position_hold

    def _compose_wait_issue(self):
        if self.last_localization_issue:
            return self.last_localization_issue
        if self.last_odom_issue:
            return self.last_odom_issue
        return self.last_observation_issue or 'Pose 超时'

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

    def make_position_setpoint(self, stamp, p_target, yaw_target):
        """速度/加速度/yaw_rate 全部屏蔽的本地位置 setpoint。

        ROS 侧数值保持 ENU，由 MAVROS 完成 ENU->NED 与 yaw 变换。
        """
        message = PositionTarget()
        message.header.stamp = stamp
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = position_only_type_mask()
        message.position.x = float(p_target[0])
        message.position.y = float(p_target[1])
        message.position.z = float(p_target[2])
        message.yaw = float(yaw_target)
        return message

    def make_local_velocity_setpoint(self, stamp, velocity, yaw):
        message = PositionTarget()
        message.header.stamp = stamp
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = velocity_absolute_yaw_type_mask()
        message.velocity.x, message.velocity.y, message.velocity.z = velocity
        message.yaw = yaw
        return message

    def publish_commands(
        self, stamp, body_velocity, yaw_rate, actual_allowed,
        position_target=None, local_velocity_target=None,
    ):
        if not self.enable_yaw_control:
            yaw_rate = 0.0

        command = TwistStamped()
        command.header.stamp = stamp
        command.header.frame_id = self.base_frame
        if position_target is not None:
            # 位置模式：cmd_vel 发布零值，不代表实际控制指令。
            command.twist.linear.x = 0.0
            command.twist.linear.y = 0.0
            command.twist.linear.z = 0.0
            command.twist.angular.z = 0.0
        else:
            command.twist.linear.x = float(body_velocity[0])
            command.twist.linear.y = float(body_velocity[1])
            command.twist.linear.z = float(body_velocity[2])
            command.twist.angular.z = float(yaw_rate)
        self._publish_if_running(self.command_pub, command)

        if position_target is not None:
            preview = self.make_position_setpoint(
                stamp, position_target[0], position_target[1]
            )
        elif local_velocity_target is not None:
            preview = self.make_local_velocity_setpoint(stamp, *local_velocity_target)
        else:
            preview = self.make_setpoint(stamp, body_velocity, yaw_rate)
        self._publish_if_running(self.preview_pub, preview)
        if self.setpoint_pub is not None:
            actual = preview if actual_allowed else self.make_setpoint(
                stamp, (0.0, 0.0, 0.0), 0.0
            )
            self._publish_if_running(self.setpoint_pub, actual)

    def set_status(self, text, motion_state):
        state_changed = (
            motion_state != self.last_motion_state
            or self.control_mode != self.last_control_mode
            or self.mode_reason != self.last_mode_reason
        )
        self.last_status = text
        self.last_motion_state = motion_state
        self.last_control_mode = self.control_mode
        self.last_mode_reason = self.mode_reason
        return state_changed

    def status_loop(self, _event):
        with self.lock:
            text = self.last_status
        if self._publish_if_running(self.status_pub, String(data=text)):
            rospy.loginfo(text)


def main():
    rospy.init_node('front_marker_control_node')
    FrontMarkerControlNode()
    rospy.spin()
