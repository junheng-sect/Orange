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
会自动回到全图搜索。

相机静态外参也可单独启动，供只运行检测与可视化的场景复用：

```bash
roslaunch front_marker_control front_camera_tf.launch
```

默认 `enable_setpoint_output=false`、`enable_yaw_control=false`。真实输出启用后要求
MAVROS connected、armed、OFFBOARD 且状态新鲜；断联、锁定、退出 OFFBOARD 或状态超时会
立即发布零 setpoint。本包不会自动解锁、切换模式或起飞。

线速度输出带三维合加速度限制。正常跟踪使用 `max_accel=0.5m/s^2`；控制器连续
`pose_timeout_sec=0.1s` 没有收到新 Pose 后，立即清除 PID 历史，并使用
`loss_decel=1.0m/s^2` 将最后输出受控减速到零。刹车期间只要上述 MAVROS 硬门仍满足，
真实 setpoint 会继续发布递减速度；重新识别后也从当前速度平滑恢复，不会阶跃到速度限幅。
偏航角速度不经过该斜率限制。

三个时间参数彼此独立：`pose_timeout_sec=0.1s` 只检查最近一次 Pose 的本机接收时间；
`max_pose_latency_sec=0.5s` 限制 Pose 从相机采集时间戳到控制节点收到时的端到端延迟；
`pid_dt_max_sec=0.5s` 只在相邻有效 Pose 的源时间戳间隔过大时重置 PID 历史。

三个参数可从完整入口直接覆盖：

```bash
roslaunch front_marker_control front_marker_tracking.launch \
  max_accel:=0.5 loss_decel:=1.0 pose_timeout_sec:=0.1 \
  max_pose_latency_sec:=0.5 pid_dt_max_sec:=0.5
```

PID 使用条件积分抗饱和：x、Yaw 的标量输出达到限幅，或 y/z 平面合成速度
`hypot(vy, vz)` 达到 `vyz_limit` 时，只冻结会继续把输出推向饱和方向的积分；
反向误差仍可消减已有积分。`vyz_limit` 只限制 YZ 平面合成速度，不是 XYZ 三轴
总速度上限；X 轴由 `vx_limit` 独立限制。
积分绝对限幅按控制组配置，以下参数默认均为 `1.0`，设为 `0.0` 可禁止对应组累积积分：

- `integral_limit_x`：前向 x PID 积分限幅
- `integral_limit_yz`：横向 y 和竖向 z PID 共用的积分限幅
- `integral_limit_yaw`：Yaw PID 积分限幅

三个线速度输出轴分别使用有状态滞回死区。已激活的轴在速度绝对值降至
`velocity_deadband_enter`（默认 `0.02m/s`）时归零；已归零的轴只有达到
`velocity_deadband`（默认 `0.03m/s`）才重新输出。两阈值必须满足
`0 <= velocity_deadband_enter <= velocity_deadband`，同时设为 `0.0` 可关闭速度死区。

三个参数均可从独立控制入口或完整跟踪入口覆盖：

```bash
roslaunch front_marker_control front_marker_tracking.launch \
  integral_limit_x:=0.5 \
  integral_limit_yz:=0.3 \
  integral_limit_yaw:=0.4
```

滞回阈值也可从独立控制入口或完整跟踪入口覆盖：

```bash
roslaunch front_marker_control front_marker_tracking.launch \
  velocity_deadband_enter:=0.02 velocity_deadband:=0.03
```

状态 `/front_marker_control/status` 使用 `TRACKING`、`BRAKING_POSE_LOSS` 和
`WAIT_POSE` 区分正常跟踪、视觉丢失刹车与完全停止。
