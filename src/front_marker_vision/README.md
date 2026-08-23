# front_marker_vision

ROS Noetic 前视 Marker 视觉包。节点只接收图像和 CameraInfo，不包含相机驱动、机体外参、
PID 或 MAVROS。

正式输出 `/front_marker/pose` 使用 `geometry_msgs/PoseStamped`：header 继承源图像，pose
表示 `^header.frame_id T_marker`。默认检测 `DICT_5X5_1000` 的 ID 33，黑色码边长
为 `0.070m`。

位姿默认使用 `IPPE_SQUARE`，对平面 Marker 的两个候选解按重投影误差和近期位姿连续性
选择；需要回退旧路径时可传入 `pose_solver:=LEGACY`。角点亚像素优化仍默认关闭。

ArUco 检测默认启用自适应 ROI。首次检测使用全图；ID33 通过轮廓门禁后，下一帧使用
Marker 外接框最大边 3 倍、且不小于 128px 的搜索区域。连续丢失时 ROI 依次扩大到
1.5 倍和 2.25 倍，第 3 次丢失后下一帧恢复全图。ROI 只作用于检测输入，CameraInfo、
Pose、TF 和完整分辨率标注图仍使用原始图像坐标。

每次有效 Pose 还会用相同时间戳广播 `pose.header.frame_id -> front_marker` 动态 TF。
可通过 `publish_marker_tf:=false` 关闭，或用 `marker_frame` 修改子坐标系名称。无有效
检测时不会广播或伪造 Marker TF。

调试输出：

- `/front_marker_vision/debug/homogeneous_transform`：同一位姿的 4x4 行主序矩阵；
- `/front_marker_vision/debug/processing_time_ms`：每帧处理耗时；
- `/front_marker_vision/image_annotated`：默认开启，默认限频 5Hz；
- `/front_marker_vision/status`：检测和拒绝原因。

独立启动：

```bash
roslaunch front_marker_vision front_marker_vision.launch
```

回退 LEGACY：

```bash
roslaunch front_marker_vision front_marker_vision.launch pose_solver:=LEGACY
```

关闭自适应 ROI、恢复每帧全图检测：

```bash
roslaunch front_marker_vision front_marker_vision.launch enable_roi:=false
```

无有效标定、图像与 CameraInfo 尺寸或 frame 不一致、未检测到目标、或位姿门禁拒绝时，
不会发布 Pose 和齐次矩阵。

节点直接接受 `mono8`、`rgb8` 和 `bgr8`。mono8 输入直接用于检测，不做每帧颜色
扩展；rgb8/bgr8 仅转换一次得到灰度图。标注图只有在达到 `annotated_rate_hz` 且存在
订阅者时才生成 BGR8，因此关闭或不订阅标注图不会触发额外的整帧颜色转换。
status 会追加当前 `roi=x,y,widthxheight miss=N`；全图搜索显示 `roi=FULL`。ROI 生效时，
标注图使用黄色矩形显示当前帧实际搜索区域。
