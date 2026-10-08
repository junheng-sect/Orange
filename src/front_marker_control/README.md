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

`/front_marker_control/control_mode` 发布当前控制模式（`std_msgs/String`，latch），
话题名由公共 YAML 话题段（`status_topic` 下方）的 `control_mode_topic` 配置。取值
`ALIGN` / `YAW_SAMPLING` / `POSE_ALIGNING` / `POS_HOLD` / `RETREAT`，只在模式变化时发布；
控制循环每周期检查一次，最迟晚一个控制周期（20 Hz）发出。不论是否启用 yaw 精对准都会
发布：到位自动读码（`barcode_arrival_trigger`）按它的 `POS_HOLD` 跳变触发，精对准与
旧的单帧锁定（`enable_yaw_control=false`）两条到位路径共用这一条话题。

## 连续采样的最终 XYZ+yaw 对准

当 `enable_position_hold=true` 且 `enable_yaw_control=true` 时，原单帧锁定由
`ALIGN -> YAW_SAMPLING -> POSE_ALIGNING -> POS_HOLD` 替代。

1. XYZ 进入现有容差且 local 位置差分速度 < 0.05 m/s，持续 1 秒后开始采样。
   进入不依赖滤波 yaw 的旧 6° 门限；采样期间视觉 XYZ 速度控制继续，绝对 yaw
   保持采样开始时的机头方向，停止使用视觉 yaw-rate。
2. Marker 按源时间戳与 Odometry 插值配对（最大插值间隔 0.1 秒），必要时等待
   后续 Odometry，最长 0.5 秒；禁止外推或接收时间降级。两路必须使用同一 ROS 时钟。
   每帧计算 local 目标航向与 Marker local 位置，而非平均机体系误差。
3. 连续 3 秒窗口：圆周中位数（样本 L1 medoid）剔除偏离 >15° 的样本；至少 60
   个保留样本，保留率 >=80%，圆周标准差 <=6°；相对保持航向修正 <=20°。
   同一保留样本集计算 Marker 平均位置和圆周平均 yaw，再按目标机头方向减去
   `target_xyz_body`，原子生成最终 XYZ+yaw。失败重新稳位采样。
4. `POSE_ALIGNING` 从实际位置和已命令的保持 yaw 开始过渡。位置 SP 使用
   静止到静止梯形/三角速度轨迹（0.1 m/s、0.5 m/s²），yaw 最短方向限速 10°/s。
   实际位置进入现有容差、yaw 距 SP <=2°、速度 <0.05 m/s 并稳定 1 秒后，
   另采独立 3 秒视觉验收窗口，采用相同样本质量门限。残差圆周均值绝对值 <=3°
   才标记 `POS_HOLD`；超限重新稳位采样。噪声超限时保持目标并重新验收，不能宣告对准。
5. `POS_HOLD` 固定最终目标，以不重叠 3 秒窗口监测视觉 yaw；超过 3°重新采样。
   位置仍按现有退出带宽与持续时间退出，新的 yaw 窗口监测替代单帧 9° yaw 退出。
   丢标时有定位可继续位置控制，但验收窗口清空；定位失效清目标、输出零速度。
   近墙后退优先，硬门控清目标。采样中 XYZ/速度越界、无效视觉或时间中断均重新计时。

接口仍为 `/mavros/setpoint_raw/local`：采样用 local ENU 速度+绝对 yaw，最终过渡与保持
用 local ENU 位置+绝对 yaw；屏蔽 yaw_rate，由 MAVROS 转换至 NED。`cmd_vel` 在采样时
仍显示 FLU 视觉速度（yaw_rate=0），在位置控制时为零，不可用其推断实际位置 SP。
`setpoint_preview` 给出完整实际模式与掩码。
`/front_marker_control/fixed_yaw_target` 为最近一次接受的最终 ENU yaw（Float64，弧度，latch）；
它是诊断值，当前是否有效应查看状态，禁止把滞留的诊断消息作为控制目标。
状态包含当前窗口数量/跨度，以及最近完成窗口的 inliers/std/correction/residual 和原因。

参数全部在 `config/front_marker_control.yaml` 的“连续源时间采样”段，保存后重启节点生效。
关闭 yaw 时绕过精对准，关闭 position hold 时保留原视觉速度控制。多帧平均降低随机噪声，
不能消除标定系统偏差，也不处理 pitch 安装偏角；3°是视觉测量标准，不是外部真值保证。

隔离验证（不会连接默认 ROS master）：

```bash
cd /home/vslam/seeker
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11389 ROS_IP=127.0.0.1
export ROS_HOME=/tmp/fmc-yaw-ros-test
catkin build front_marker_control
catkin run_tests front_marker_control
catkin_test_results build/front_marker_control/test_results
```

集成测试使用 `/test_*` 输入输出话题和合成位置反馈，测试用短窗口由 test launch 显式覆盖；
纯 Python 状态机测试使用完整默认 3 秒窗口。测试通过不代表 FCU/实飞验证。

## OV13855 + AprilTag 独立链路

新入口使用 `ov13855_cam` 和 `ov13855_apriltag_vision`，外参入口为
`config/ov13855_camera_tf.yaml`（安装数值沿用旧相机，光学系为 `ov13855_optical_frame`）。

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
roslaunch front_marker_control ov13855_marker_tracking.launch
```

tag 边长取自标签版式文件，默认 `layout:=qr_top_tag_bottom`（`barcode_reader_camera/config/layouts/`），
换标签时传 `layout:=<名字>`。

旧 `front_marker_tracking.launch` 保持不变。退出新入口后可用旧入口回退；
两者不能同时运行，因为共用 `/front_marker/pose` 和 `front_marker` TF。
新相机标定采用参考配置，分辨率暂假定为 2560×1440，尺度实测见新视觉包 `VALIDATION.md`。
