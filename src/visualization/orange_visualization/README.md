# orange_visualization

Orange Pi 无人机的 TF 与 RViz 可视化包（包名 10-21 由旧 SEEKER 板沿用的 `seeker_visualization` 改来）。
该包不会修改 MAVROS 参数。RViz 看的是这条 TF 链，机体那一段有两种来源，由 `publish_mavros_tf` 选：

```text
drone_odom -> ... -> drone_base -> usb_camera_optical_frame -> front_marker   # 默认：FAST-LIO 重力桥
px4_map    ->        drone_base -> usb_camera_optical_frame -> front_marker   # publish_mavros_tf:=true
```

## TF 链定义

- 默认（`publish_mavros_tf:=false`）：`drone_base` 来自 FAST-LIO 链路（`drone_odom -> camera_init -> body -> drone_base`），
  本包不发机体 TF，RViz Fixed Frame 和标签参考系都是 `drone_odom`。orange 上跑 fusion.sh 时就是这种情况。
- `publish_mavros_tf:=true` 时 `px4_map -> drone_base` 由 `mavros_odom_tf_node.py` 转发飞控融合外部位姿（orange 上是 FAST-LIO）后的
  MAVROS Odometry。节点严格校验 `frame_id=px4_map`、`child_frame_id=drone_base`、
  时间戳、数值有限性和单位四元数，只有合法消息才会广播，不提供零位姿兜底；
  MAVROS 未连接或数据非法时，该 TF 边不存在。
  帧名要求 MAVROS 经 mid360s_ws 的 `scripts/launch/mavros.launch` 启动（10-21 起是 `px4_map` / `drone_base`）；
  直接 `roslaunch mavros px4.launch` 时消息里还是 `map` / `base_link`，会被拒收。
  只在没跑 FAST-LIO 时打开：重力桥也发 `body -> drone_base`，两边同时发会让 `drone_base` 有两个父帧。
- `drone_base -> usb_camera_optical_frame`：由 `front_marker_control` 的
  `front_camera_tf.launch` 发布的静态外参。相机位于 `drone_base` 原点
  （机体重心）正前方 0.15 m，光轴朝机头方向。
- `usb_camera_optical_frame -> front_marker`：由 `front_marker_vision_node`
  在每次有效 ArUco 检测后动态发布，随 `/front_marker/pose` 实时更新。

Marker 不再由静态 TF 预设在固定系中，而是由上述 TF 链实时计算出其在
固定系（`drone_odom` 或 `px4_map`）中的位置。当实体 Marker 静止时，其固定系估计应近似稳定，只含
视觉解算和里程计噪声；Marker 丢失时该动态 TF 停止更新。因此 RViz 中
`front_marker` 的位姿反映的是"飞行器看起来的真实状态"，可直接用于观察
里程计漂移与视觉定位的一致性。

## 启动

在板端图形桌面终端启动完整检测与可视化：

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
source devel/setup.bash
roslaunch orange_visualization front_marker_rviz.launch
```

默认需要 FAST-LIO 链路已在运行（提供 `drone_odom -> drone_base`）。没跑 FAST-LIO、只看 MAVROS 位姿时：

```bash
roslaunch orange_visualization front_marker_rviz.launch publish_mavros_tf:=true
```

如果完整的 `front_marker_tracking.launch`（相机驱动、视觉节点、控制节点、
相机静态 TF 和 Marker 动态 TF）已经在外部运行，只启动 RViz（和标签）：

```bash
roslaunch orange_visualization front_marker_rviz.launch start_tracking:=false
```

`start_tracking:=false` 只适用于上述场景；此时本包不再发布相机静态 TF，
避免同一条 TF 边出现两个发布源。

两种入口都保持控制输出关闭（RViz 场景在
`front_marker_control/config/front_marker_control_rviz.yaml` 中关闭 yaw 控制和
控制输出）。坐标系名称由 `tf_frame_labels_node.py` 以带偏移的文字 Marker 显示，
并用细引线指向对应原点，避免 `drone_base` 与 `usb_camera_optical_frame` 等重合
原点的文字互相遮挡。可按需关闭：

```bash
roslaunch orange_visualization front_marker_rviz.launch show_frame_labels:=false
```

RViz 默认同时启用 `Image Raw` Display，通过 `raw` transport 订阅
`/usb_cam/image_raw`，队列长度为 2，以减少调试画面的排队延迟。

## 参数入口

`mavros_odom_tf_node` 与 `tf_frame_labels_node` 的运行参数统一在
`config/orange_visualization.yaml` 中维护（里程计话题、TF 坐标系名、标签偏移/
文字高度/发布频率等，含中文注释和单位），修改后重启 launch 生效。要临时换用
自定义配置：

```bash
roslaunch orange_visualization front_marker_rviz.launch \
  config_file:=/tmp/my_visualization.yaml
```

启动接口 `start_tracking`、`show_frame_labels`、`publish_mavros_tf`、`fixed_frame`、`rviz_config` 仍由命令行提供；
`fixed_frame` 默认随 `publish_mavros_tf` 取 `drone_odom` / `px4_map`，launch 会用它覆盖 RViz（`-f`）和标签节点的参考系。

## 注意事项

- 不要在无图形 `DISPLAY` 的 systemd/SSH 会话中自动启动 RViz。
- `publish_mavros_tf:=true` 且 MAVROS 未提供合法 Odometry 时，RViz 以 `px4_map` 为 Fixed Frame 提示缺少
  变换属于预期行为，此时整条链都无法显示；默认模式下 FAST-LIO 没跑也是同样现象（缺 `drone_odom`）。
- MAVROS Odometry 与图像时间戳需处于同一 ROS 时间基准，否则
  `px4_map -> front_marker` 的 TF 查询会出现外推错误。
