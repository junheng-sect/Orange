# seeker_visualization

SEEKER 板端 TF 与 RViz 可视化包。该包不会修改 MAVROS 参数，而是校验
`/mavros/local_position/odom` 后广播唯一一条真实 TF 链：

```text
map -> base_link -> usb_camera_optical_frame -> front_marker
```

## TF 链定义

- `map -> base_link`：由 `mavros_odom_tf_node.py` 转发飞控融合 VIO 后的
  MAVROS Odometry。节点严格校验 `frame_id=map`、`child_frame_id=base_link`、
  时间戳、数值有限性和单位四元数，只有合法消息才会广播，不提供零位姿兜底；
  MAVROS 未连接或数据非法时，该 TF 边不存在。
- `base_link -> usb_camera_optical_frame`：由 `front_marker_control` 的
  `front_camera_tf.launch` 发布的静态外参。相机位于 `base_link` 原点
  （机体重心）正前方 0.15 m，光轴朝机头方向。
- `usb_camera_optical_frame -> front_marker`：由 `front_marker_vision_node`
  在每次有效 ArUco 检测后动态发布，随 `/front_marker/pose` 实时更新。

Marker 不再由静态 TF 预设在 `map` 中，而是由上述 TF 链实时计算出其在
`map` 系中的位置。当实体 Marker 静止时，其 map 系估计应近似稳定，只含
视觉解算和里程计噪声；Marker 丢失时该动态 TF 停止更新。因此 RViz 中
`front_marker` 的位姿反映的是"飞行器看起来的真实状态"，可直接用于观察
VIO 漂移与视觉定位的一致性。

## 启动

在板端图形桌面终端启动完整检测与可视化：

```bash
source /opt/ros/noetic/setup.bash
cd /home/vslam/seeker
source devel/setup.bash
roslaunch seeker_visualization front_marker_rviz.launch
```

如果完整的 `front_marker_tracking.launch`（相机驱动、视觉节点、控制节点、
相机静态 TF 和 Marker 动态 TF）已经在外部运行，只补 MAVROS TF 并启动 RViz：

```bash
roslaunch seeker_visualization front_marker_rviz.launch start_tracking:=false
```

`start_tracking:=false` 只适用于上述场景；此时本包不再发布相机静态 TF，
避免同一条 TF 边出现两个发布源。

两种入口都保持控制输出关闭（RViz 场景在
`front_marker_control/config/front_marker_control_rviz.yaml` 中关闭 yaw 控制和
控制输出）。坐标系名称由 `tf_frame_labels_node.py` 以带偏移的文字 Marker 显示，
并用细引线指向对应原点，避免 `base_link` 与 `usb_camera_optical_frame` 等重合
原点的文字互相遮挡。可按需关闭：

```bash
roslaunch seeker_visualization front_marker_rviz.launch show_frame_labels:=false
```

RViz 默认同时启用 `Image Raw` Display，通过 `raw` transport 订阅
`/usb_cam/image_raw`，队列长度为 2，以减少调试画面的排队延迟。

## 参数入口

`mavros_odom_tf_node` 与 `tf_frame_labels_node` 的运行参数统一在
`config/seeker_visualization.yaml` 中维护（里程计话题、TF 坐标系名、标签偏移/
文字高度/发布频率等，含中文注释和单位），修改后重启 launch 生效。要临时换用
自定义配置：

```bash
roslaunch seeker_visualization front_marker_rviz.launch \
  config_file:=/tmp/my_visualization.yaml
```

启动接口 `start_tracking`、`show_frame_labels`、`rviz_config` 仍由命令行提供。

## 注意事项

- 不要在无图形 `DISPLAY` 的 systemd/SSH 会话中自动启动 RViz。
- 若 MAVROS 未提供合法 Odometry，RViz 以 `map` 为 Fixed Frame 时提示缺少
  变换属于预期行为，此时整条链都无法显示。
- MAVROS Odometry 与图像时间戳需处于同一 ROS 时间基准，否则
  `map -> front_marker` 的 TF 查询会出现外推错误。
