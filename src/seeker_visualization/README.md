# seeker_visualization

SEEKER 板端 TF 与 RViz 可视化包。该包不会修改 MAVROS 参数，而是校验
`/mavros/local_position/odom` 后广播：

```text
map -> base_link -> usb_camera_optical_frame -> front_marker
```

`map -> base_link` 只有在 MAVROS 已连接且收到合法 Odometry 时才会出现，不会使用零位姿
兜底。`base_link -> usb_camera_optical_frame` 复用 `front_marker_control` 的固定外参，
`usb_camera_optical_frame -> front_marker` 来自有效 Marker 检测。

MAVROS 暂时不可用时，可以显式打开临时调试 TF。该模式把 `base_link` 固定在 `map`
正上方 0.10 m，并停用动态 MAVROS Odometry TF，避免同一条 TF 边由两个节点发布：

```bash
roslaunch seeker_visualization front_marker_rviz.launch debug_static_map_to_base:=true
```

该开关默认关闭，只用于打通和检查可视化 TF 链，不能代表飞行器真实位姿。

RViz 中四个坐标轴的单轴长度统一为 0.10 m。坐标系名称由
`tf_frame_labels_node.py` 以带偏移的文字 Marker 显示，并用细引线指向对应原点，避免
`base_link` 与 `usb_camera_optical_frame` 等重合原点的文字互相遮挡。可按需关闭：

```bash
roslaunch seeker_visualization front_marker_rviz.launch show_frame_labels:=false
```

RViz 默认同时启用 `Image Raw` Display，通过 `raw` transport 订阅
`/usb_cam/image_raw`，队列长度为 2，以减少调试画面的排队延迟。

在板端图形桌面终端启动完整检测与可视化：

```bash
source /opt/ros/noetic/setup.bash
cd /home/vslam/seeker
source devel/setup.bash
roslaunch seeker_visualization front_marker_rviz.launch
```

如果 `ov9281_usb_cam` 和 `front_marker_vision` 已经分别启动，只补齐 TF 并启动 RViz：

```bash
roslaunch seeker_visualization front_marker_rviz.launch start_tracking:=false
```

两种入口都保持 `enable_setpoint_output=false`。不要在无图形 `DISPLAY` 的 systemd/SSH
会话中自动启动 RViz；若 MAVROS 未连接，RViz 以 `map` 为 Fixed Frame 时提示缺少变换是
预期行为。
