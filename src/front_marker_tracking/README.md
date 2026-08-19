# front_marker_tracking（ROS Noetic USB/CSI 实机版）

该包保留 SEEKER 上正装朝前的 DECXIN/OV9281 USB 相机链路，并为 Orange Pi 5 Max
新增 CAM1/OV13855 CSI 图像发布节点。Marker 检测和控制逻辑仍用于检测嵌套
`DICT_5X5_1000` Marker 的外层 ID33（黑色码边长 `0.193m`），并生成相机居中、
码面正对、距离 `0.8m` 的控制预览。内层 ID42（`0.025m`）只用于近距离视觉确认，
不参与位姿回退或控制。

原 USB 链路使用 `640x480`、MJPEG、30fps，并由 `usb_cam` 发布：

- `/usb_cam/image_raw`
- `/usb_cam/camera_info`
- frame_id：`usb_camera_optical_frame`

## Orange Pi 5 Max CAM1 CSI

Orange Pi 入口使用不会随 `/dev/videoN` 编号变化的 `/dev/video-camera0`。节点启动时
会解析该软链接并检查 V4L2 产品名必须为 `rkisp_mainpath`；原始 Bayer/CIF 节点
（例如 `/dev/video0`）会被拒绝。当前实机链路为：

```text
OV13855 CAM1 -> MIPI CSI-2 -> RKCIF -> RKISP -> /dev/video-camera0 -> /dev/video11
```

构建和启动：

```bash
cd /home/orangepi/project/orange
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
roslaunch front_marker_tracking front_marker_csi_camera.launch
```

这条 CAM1 默认命令会一次启动：

- `front_marker_csi_camera_node`
- `front_marker_detector_node`
- `front_marker_tracking_node`

跟踪节点默认固定为预览模式：`enable_setpoint_output=false`、
`enable_yaw_control=false`，不会创建 `/mavros/setpoint_raw/local` 发布者。只需要相机时使用：

```bash
roslaunch front_marker_tracking front_marker_csi_camera.launch \
  start_tracking_stack:=false
```

默认 GStreamer 管线请求 NV12 720x576@15fps，经 `videoconvert` 发布：

- `/cam1/image_raw`：`bgr8`、frame_id `cam1_optical_frame`
- `/cam1/camera_info`

CSI 节点只发布标准 raw Image 和 CameraInfo，不自动广告 `compressedDepth`、`compressed`
或 `theora` 传输。`rqt_image_view` 中请选择 `/cam1/image_raw`；彩色 `bgr8` 不能使用
仅面向 16-bit/32-bit 深度图的 `compressedDepth`。

RKAIQ 自动曝光可能通过增加 OV13855 的 `vertical_blanking` 延长帧周期，因此 caps 仍为
15fps 时，暗光或逆光场景下的实际帧率可能更低；节点不会擅自覆盖 3A 的曝光和帧长控制。

检查图像链路：

```bash
rostopic hz /cam1/image_raw
rostopic echo -n 1 /cam1/image_raw/width
rostopic echo -n 1 /cam1/image_raw/height
rostopic echo -n 1 /cam1/image_raw/encoding
rostopic echo -n 1 /cam1/image_raw/header/frame_id
```

### 标定门禁

OV13855 720x576 尚未完成 ROS 相机标定时，`camera_info_url` 必须留空。相机节点只发布
`K[0]=0` 的标准未标定 CameraInfo，不会伪造内参。现有
`config/ov9281_640x480.yaml` 属于另一颗相机和分辨率，禁止用于 CAM1。

默认 CAM1 命令在未标定时仍会运行 ArUco 二维识别，并在彩色
`/front_marker_tracking/image_annotated` 上绘制 Marker 边框和 ID。识别到目标时 detector
状态为 `DETECTED_2D: ID33 | POSE_DISABLED: INVALID_CAMERA_INFO...`；没有目标时为
`WAIT_MARKER_2D`。二维识别不使用伪造内参，tracking 仍每秒输出
`PREVIEW_ONLY | WAIT_MARKER`，且 `/front_marker_tracking/transform` 无消息。只有加载与
720×576 对应的真实标定后才允许计算并发布公制位姿。

如需单独运行 detection-only，先用 `start_tracking_stack:=false` 启动相机，再执行：

```bash
roslaunch front_marker_tracking front_marker_detection.launch \
  use_usb_cam:=false \
  image_topic:=/cam1/image_raw \
  camera_info_topic:=/cam1/camera_info \
  camera_frame_id:=cam1_optical_frame \
  image_width:=720 image_height:=576
```

取得真实的 OV13855 720x576 标定文件后，把它传给默认 CAM1 命令即可同时启动完整安全预览链路：

```bash
roslaunch front_marker_tracking front_marker_csi_camera.launch \
  camera_info_url:=file:///absolute/path/ov13855_cam1_720x576.yaml
```

如改用分开的 launch，控制预览也必须保持真实输出关闭：

```bash
roslaunch front_marker_tracking front_marker_tracking.launch \
  use_usb_cam:=false \
  image_topic:=/cam1/image_raw \
  camera_info_topic:=/cam1/camera_info \
  camera_frame_id:=cam1_optical_frame \
  image_width:=720 image_height:=576 \
  enable_setpoint_output:=false enable_yaw_control:=false
```

## 坐标与外参约定

默认模式不使用实测 cam-IMU 外参，也不启动 `front_marker_base_transform_node`。相机
必须正装朝前，画面上方对应机体上方；控制只采用标准 optical 到 FLU 的轴置换：

```text
camera optical: x right, y down, z forward
FLU:            x forward, y left, z up

R_flu_optical = [ 0  0  1
                 -1  0  0
                  0 -1  0 ]
t_flu_optical = [0, 0, 0] m
```

因此默认 `target_xyz_body=[0.8,0,0]m` 等价于相机系 `[0,0,0.8]m`。这里的 FLU 只定义
控制轴方向，零平移表示相机光心就是对准原点；默认链路不会发布声称位于 IMU 原点的
`/front_marker_tracking/right/transform_base_marker` 或 `marker_pose_base`。

## 相机标定

`config/ov9281_640x480.yaml` 使用：

```text
K = [741.67267,   0,       290.60259
       0,         741.44273,217.48395
       0,           0,       1]
D = [0.067792, 0.004847, 0.001479, -0.000168, 0.0]
```

USB 设备使用不会随 `/dev/videoN` 变化的路径：

```text
/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0
```

`video-index1` 是 UVC metadata 节点，不能作为图像源。

## 构建

```bash
cd /home/vslam/rk2_ws
source /opt/ros/noetic/setup.bash
source /home/vslam/catkin_ws_vins/devel/setup.bash
catkin_make -j2
source devel/setup.bash
```

## 只检测

```bash
source /opt/ros/noetic/setup.bash
source /home/vslam/catkin_ws_vins/devel/setup.bash
source /home/vslam/rk2_ws/devel/setup.bash
roslaunch front_marker_tracking front_marker_detection.launch
```

检查：

```bash
rostopic hz /usb_cam/image_raw
rostopic echo -n 1 /usb_cam/camera_info
rostopic echo /front_marker_tracking/detector_status
rostopic hz /front_marker_tracking/transform
```

如果图像和 CameraInfo 由外部节点提供：

```bash
roslaunch front_marker_tracking front_marker_detection.launch \
  use_usb_cam:=false \
  image_topic:=/custom/image_raw \
  camera_info_topic:=/custom/camera_info \
  camera_frame_id:=usb_camera_optical_frame
```

## 跟踪与安全输出

安全预览：

```bash
roslaunch front_marker_tracking front_marker_tracking.launch \
  enable_setpoint_output:=false \
  enable_yaw_control:=false \
  publish_annotated:=false
```

`enable_yaw_control`默认为`false`：继续发布原始/滤波Yaw误差，但所有预览和真实
setpoint的`yaw_rate`保持为零，`ALIGNED`只检查XYZ。设为`true`并重启节点后恢复
Yaw PID和Yaw对准条件。

只有显式设置 `enable_setpoint_output:=true` 才创建
`/mavros/setpoint_raw/local` 发布者。非零输出仍必须同时满足：

- MAVROS 状态新鲜且 `connected=true`
- `armed=true`
- 模式严格等于 `OFFBOARD`
- ID33 位姿和源图像时间戳有效且未超时

本包不会自动解锁、切换模式或起飞。视觉失效、时间戳倒退或状态门禁不满足时，真实
输出保持为零。

主要话题：

- `/front_marker_tracking/transform`
- `/front_marker_tracking/detector_status`
- `/front_marker_tracking/image_annotated`（可关闭）
- `/front_marker_tracking/marker_pose_camera`
- `/front_marker_tracking/position_error_camera`
- `/front_marker_tracking/yaw_error`
- `/front_marker_tracking/error_flu`
- `/front_marker_tracking/error_flu_filtered`
- `/front_marker_tracking/cmd_vel_camera`
- `/front_marker_tracking/cmd_vel_flu`
- `/front_marker_tracking/setpoint_preview`
- `/front_marker_tracking/status`

## 开机自启动

正式板端由 `seeker_utils/launch/seeker_autostart.launch` 启动包装脚本
`front_marker_tracking_autostart.sh`。包装脚本加载 `/home/vslam/rk2_ws/devel/setup.bash`
后启动本包，当前正式配置会传入 `enable_setpoint_output:=true`和
`enable_yaw_control:=false`，但不会绕过上述门禁。

## Legacy 双目工具

以下入口保留用于历史回归，不参与 USB 相机默认链路：

- `front_marker_dual_observation.launch`
- `front_marker_right_ab.launch`
- `config/front_left_cam0.yaml`
- `config/front_right_cam1.yaml`

它们仍使用 `/front/left`、`/front/right`、640x768、cam0/cam1 和原标定外参。不要与
默认 USB launch 同时启动同名检测或控制节点。

## 测试

```bash
cd /home/vslam/rk2_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_front_marker_tracking
catkin_test_results
```

测试覆盖 640x480 CameraInfo、拒绝帧不发布、标准 optical→FLU 轴符号、0.8m 零误差、
PID 每视觉帧只更新一次、20Hz 心跳、视觉超时归零和 MAVROS 发布隔离。
