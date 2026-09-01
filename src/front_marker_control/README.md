# front_marker_control

ROS Noetic 前视 Marker 运动控制包。唯一视觉输入是标准
`geometry_msgs/PoseStamped` `/front_marker/pose`；节点不依赖视觉包的自定义消息、OpenCV
或 CameraInfo。

节点使用 TF2 将输入 Pose 按源时间戳转换到 `base_link`，再计算目标位置
`[0.8, 0.0, 0.0]` 的误差和机体 FLU 速度。独立启动时必须由外部提供
`base_link ← pose.header.frame_id` TF：

```bash
roslaunch front_marker_control front_marker_control.launch
```

完整 OV9281、TF、Vision、Control 入口：

```bash
roslaunch front_marker_control front_marker_tracking.launch
```

完整入口默认令 OV9281 保持 MJPEG 采集，同时将解码后的 raw 图像发布为 `mono8`，避免
Marker 检测前的 RGB 中间图；位姿求解默认使用 `IPPE_SQUARE`，角点亚像素优化仍关闭，
并默认启用 ID33 自适应 ROI。可用 `camera_output_encoding:=rgb8`、
`pose_solver:=LEGACY` 或 `enable_roi:=false` 分别恢复对应兼容模式：

```bash
roslaunch front_marker_control front_marker_tracking.launch \
  camera_output_encoding:=rgb8 pose_solver:=LEGACY enable_roi:=false
```

ROI 只缩小 ArUco 检测输入，不裁剪相机 raw/compressed 或标注图；连续 3 帧丢失目标后
会自动回到全图搜索。完整入口还会透传 `min_marker_perimeter_rate`（默认 `0.075`）；
视觉节点会结合当前 ROI 大小和 12px 轮廓门禁动态提高实际阈值，从检测早期跳过不可能
通过后续门禁的极小候选。

相机静态外参也可单独启动，供只运行检测与可视化的场景复用：

```bash
roslaunch front_marker_control front_camera_tf.launch
```

默认 `enable_setpoint_output=false`、`enable_yaw_control=false`。真实输出启用后要求
MAVROS connected、armed、OFFBOARD 且状态新鲜；断联、锁定、退出 OFFBOARD 或状态超时会
立即发布零 setpoint。本包不会自动解锁、切换模式或起飞。

线速度输出带三维合加速度限制。正常跟踪使用 `max_accel=0.5m/s^2`；控制器连续
`pose_timeout_sec=0.1s` 没有收到新 Pose 后，立即清除控制器状态，并使用
`loss_decel=1.0m/s^2` 将最后输出受控减速到零。刹车期间只要上述 MAVROS 硬门仍满足，
真实 setpoint 会继续发布递减速度；重新识别后也从当前速度平滑恢复，不会阶跃到速度限幅。
偏航角速度不经过该斜率限制。

三个时间参数彼此独立：`pose_timeout_sec=0.1s` 只检查最近一次 Pose 的本机接收时间；
`max_pose_latency_sec=0.5s` 限制 Pose 从相机采集时间戳到控制节点收到时的端到端延迟；
`filter_dt_max_sec=0.5s` 只在相邻有效 Pose 的源时间戳间隔过大时重置 Yaw 低通与
速度死区状态。

三个参数可从完整入口直接覆盖：

```bash
roslaunch front_marker_control front_marker_tracking.launch \
  max_accel:=0.5 loss_decel:=1.0 pose_timeout_sec:=0.1 \
  max_pose_latency_sec:=0.5 filter_dt_max_sec:=0.5
```

控制器为纯比例控制：`v = Kp · e`，`yaw_rate = kp_yaw · e_yaw,filtered`，
不含积分与微分项，因此也不存在积分抗饱和。三个增益 `kp_x`、`kp_yz`、`kp_yaw`
在 `config/front_marker_control.yaml` 中配置。`vyz_limit` 只限制 YZ 平面合成速度，
不是 XYZ 三轴总速度上限；X 轴由 `vx_limit` 独立限制，Yaw 角速度由
`yaw_rate_limit` 限幅。

XYZ 位置误差在进入比例控制、Z 误差死区、速度限幅和速度滞回死区之前，分别经过
一阶低通滤波。X 轴时间常数由 `position_lpf_tau_x` 配置，默认 `0.05s`；Y、Z
使用两个独立滤波器并共享 `position_lpf_tau_yz`，默认 `0.10s`。Yaw 的
`yaw_lpf_tau` 默认 `0.20s`。任一时间常数设为 `0.0` 可关闭对应滤波；滤波首帧
直接采用当前误差，视觉失效或时间戳异常时会清空历史状态。

三个线速度输出轴分别使用有状态滞回死区。已激活的轴在速度绝对值降至
`velocity_deadband_enter`（默认 `0.02m/s`）时归零；已归零的轴只有达到
`velocity_deadband`（默认 `0.03m/s`）才重新输出。两阈值必须满足
`0 <= velocity_deadband_enter <= velocity_deadband`，同时设为 `0.0` 可关闭速度死区。

滞回阈值可从独立控制入口或完整跟踪入口覆盖：

```bash
roslaunch front_marker_control front_marker_tracking.launch \
  velocity_deadband_enter:=0.02 velocity_deadband:=0.03
```

状态 `/front_marker_control/status` 使用 `TRACKING`、`BRAKING_POSE_LOSS` 和
`WAIT_POSE` 区分正常跟踪、视觉丢失刹车与完全停止。
