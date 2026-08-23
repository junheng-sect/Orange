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
Marker 检测前的 RGB 中间图；位姿求解默认使用 `IPPE_SQUARE`，角点亚像素优化仍关闭。
可用 `camera_output_encoding:=rgb8` 或 `pose_solver:=LEGACY` 分别恢复对应兼容模式：

```bash
roslaunch front_marker_control front_marker_tracking.launch \
  camera_output_encoding:=rgb8 pose_solver:=LEGACY
```

相机静态外参也可单独启动，供只运行检测与可视化的场景复用：

```bash
roslaunch front_marker_control front_camera_tf.launch
```

默认 `enable_setpoint_output=false`、`enable_yaw_control=false`。真实输出启用后仍要求
MAVROS connected、armed、OFFBOARD、状态新鲜且 Pose/TF 新鲜；任一条件不满足时只发布
零 setpoint。本包不会自动解锁、切换模式或起飞。
