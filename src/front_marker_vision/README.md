# front_marker_vision

ROS Noetic 前视 Marker 视觉包。节点只接收图像和 CameraInfo，不包含相机驱动、机体外参、
PID 或 MAVROS。

正式输出 `/front_marker/pose` 使用 `geometry_msgs/PoseStamped`：header 继承源图像，pose
表示 `^header.frame_id T_marker`。默认检测 `DICT_5X5_1000` 的 ID 33，黑色码边长
为 `0.070m`。

调试输出：

- `/front_marker_vision/debug/homogeneous_transform`：同一位姿的 4x4 行主序矩阵；
- `/front_marker_vision/debug/processing_time_ms`：每帧处理耗时；
- `/front_marker_vision/image_annotated`：默认开启，默认限频 5Hz；
- `/front_marker_vision/status`：检测和拒绝原因。

独立启动：

```bash
roslaunch front_marker_vision front_marker_vision.launch
```

无有效标定、图像与 CameraInfo 尺寸或 frame 不一致、未检测到目标、或位姿门禁拒绝时，
不会发布 Pose 和齐次矩阵。
