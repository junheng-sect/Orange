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

## 安全与失效策略

- 默认 `enable_setpoint_output=false`、`enable_yaw_control=true`。真实输出启用后
  要求 MAVROS connected、armed、OFFBOARD 且状态新鲜；断联、锁定、退出 OFFBOARD
  或状态超时会立即发布零 setpoint。本包不会自动解锁、切换模式或起飞。
- Marker 位姿时间戳异常、四元数非有限或不可归一化时拒绝输入。
- `enable_yaw_control=false` 时保留原有机体系 FLU XYZ 跟踪，同时屏蔽 yaw 与
  yaw-rate 字段。

## 启动

所有运行参数统一在 `config/front_marker_control.yaml` 中维护（含中文注释和单位），
修改后重启 launch 生效；launch 不再透传调参参数。场景差异用“公共 YAML＋场景
YAML”表达，加载顺序固定为先公共、后场景：

- `config/front_marker_control.yaml` —— 公共配置（跟踪场景的默认值）。
- `config/front_marker_control_rviz.yaml` —— RViz 可视化场景差异项
  （关闭 `enable_yaw_control` 和 `enable_setpoint_output`，由
  `seeker_visualization` 入口自动叠加）。

只启动控制节点（需要外部提供 `base_link <- pose.header.frame_id` TF）：

```bash
roslaunch front_marker_control front_marker_control.launch
```

ROI 只缩小 ArUco 检测输入，不裁剪相机 raw/compressed 或标注图；连续 3 帧丢失目标后
会自动回到全图搜索。完整入口还会透传 `min_marker_perimeter_rate`（默认 `0.075`）；
视觉节点会结合当前 ROI 大小和 12px 轮廓门禁动态提高实际阈值，从检测早期跳过不可能
通过后续门禁的极小候选。

相机静态外参也可单独启动，供只运行检测与可视化的场景复用：

```bash
roslaunch front_marker_control front_camera_tf.launch
```

完整入口默认令 OV9281 保持 MJPEG 采集，同时将解码后的 raw 图像发布为 `mono8`
（相机场景差异见 `ov9281_usb_cam/config/ov9281_mono8.yaml`）；位姿求解默认使用
`IPPE_SQUARE`，并默认启用 ID33 自适应 ROI。要临时恢复兼容模式，改各包对应 YAML
（如 `ov9281_rgb8.yaml`、`front_marker_vision.yaml` 的 `pose_solver`/`enable_roi`）
或用 `scene_config`/`config_file` 传入自己的配置文件，例如：

```bash
roslaunch front_marker_control front_marker_control.launch \
  config_file:=/tmp/my_control.yaml scene_config:=/tmp/my_scene.yaml
roslaunch front_marker_control front_marker_tracking.launch \
  control_config:=/tmp/my_control.yaml vision_config:=/tmp/my_vision.yaml
```

相机静态 TF（`base_link -> usb_camera_optical_frame`）数值在
`config/front_camera_tf.yaml` 中维护，由 `front_camera_static_tf_node.py` 读取并
发布，不再使用 launch 命令行数值；节点会拒绝非法配置（非有限值、非单位四元数、
空或相同坐标系）。

## 主要参数

以下参数全部在 `config/front_marker_control.yaml` 中修改（节点代码默认值仅作
兜底）：

```text
position_lpf_tau_x                0.05
position_lpf_tau_y                0.00
position_lpf_tau_z                0.00
```

对齐容差 `distance_tolerance_m`/`lateral_tolerance_m`/
`vertical_tolerance_m`、`yaw_tolerance_deg` 与 `aligned_hold_sec` 同样在公共
YAML 中。

XYZ 位置一阶低通时间常数（秒）：`position_lpf_tau_x/y/z` 在公共 YAML 中，`tau=0.0`
表示该轴不滤波。输入与安全时间参数还包括 `pose_timeout_sec`、
`max_pose_latency_sec`、`filter_dt_max_sec`、`state_timeout_sec`、
`future_stamp_tolerance_sec` 和 `tf_lookup_timeout_sec`。XYZ 位置环增益
`kp_x`/`kp_y`/`kp_z`、速度限幅 `vyz_limit`/`vx_limit`、速度指令滞回死区按轴独立
（单位 m/s，`off` = 低于则停止输出，`on` = 高于则开始输出，需 `on >= off`）：
`velocity_deadband_off_x/on_x`（`0.010/0.015`）、`velocity_deadband_off_y/on_y`
（`0.004/0.007`）、`velocity_deadband_off_z/on_z`（`0.003/0.006`），以及全部
输入/输出话题名，均在公共 YAML 中。

例如，把位置环增益或速度死区改掉：直接修改 `config/front_marker_control.yaml`
中对应项并重启 launch（不再支持命令行逐项覆盖）。


控制器为纯比例控制：`v = Kp · e`，`yaw_rate = kp_yaw · e_yaw,filtered`，
不含积分与微分项，因此也不存在积分抗饱和。位置增益 `kp_x`、`kp_y`、`kp_z`、
偏航增益 `kp_yaw` 和其他控制参数均在 `config/front_marker_control.yaml` 中配置。
`vyz_limit` 只限制 YZ 平面合成速度，
不是 XYZ 三轴总速度上限；X 轴由 `vx_limit` 独立限制，Yaw 角速度由
`yaw_rate_limit` 限幅。

XYZ 位置误差在进入比例控制、速度限幅和速度滞回死区之前，分别经过
一阶低通滤波。X、Y、Z 三轴分别由 `position_lpf_tau_x/y/z` 配置，当前默认为
`0.05/0.00/0.00s`，在 `config/front_marker_control.yaml` 中修改。Yaw 的 `yaw_lpf_tau` 默认
`0.20s`，仍由 `config/front_marker_control.yaml` 配置。任一时间常数设为 `0.0`
可关闭对应滤波；滤波首帧
直接采用当前误差，视觉失效或时间戳异常时会清空历史状态。

三个线速度输出轴分别使用独立的有状态滞回死区（`off` = 低于则停止输出，
`on` = 高于则开始输出，需 `on >= off`）：`velocity_deadband_off_x/on_x`、
`velocity_deadband_off_y/on_y`、`velocity_deadband_off_z/on_z`。默认推荐值
`0.010/0.015`（x）、`0.004/0.007`（y）、`0.003/0.006`（z），同时设为
`0.0/0.0` 可关闭对应轴速度死区。

滞回阈值在 `config/front_marker_control.yaml` 中修改。

状态 `/front_marker_control/status` 使用 `TRACKING`、`BRAKING_POSE_LOSS` 和
`WAIT_POSE` 区分正常跟踪、视觉丢失刹车与完全停止。
