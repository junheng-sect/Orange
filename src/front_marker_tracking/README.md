# front_marker_tracking

ROS Noetic 下的 OV9281 USB 单相机 ArUco 检测与跟踪包。相机、检测器和跟踪控制由
本包的 launch 启动；跟踪控制保留 MAVROS 状态门禁，默认不发布真实 setpoint。

## 相机输入

相机型号固定为 OV9281，使用稳定的 UVC 设备路径：

```text
/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0
```

输入链固定使用：

- `ov9281_usb_cam`、MJPEG、`mmap`
- frame_id：`usb_camera_optical_frame`
- 图像：`/usb_cam/image_raw`
- 标定：`/usb_cam/camera_info`

默认 profile 为 `640x480 @ 30fps`：

```text
camera_name:    ov9281_640x480
camera_info_url: config/ov9281_640x480.yaml
```

相机节点始终由 launch 启动，不提供关闭相机或接入其他图像源的开关。
`ov9281_usb_cam` 基于官方 `usb_cam` 0.3.7，但使用独立包名，不覆盖系统 apt 包。
其 FFmpeg `SwsContext` 会跨帧复用，所以 `yuv422p -> rgb24` 无硬件加速提示最多在
上下文首次创建时出现一次；实际颜色转换仍由 CPU 完成。

## 标定 profile

仓库保留三份 OV9281 标定：

| 分辨率 | camera_name | 标定文件 |
| --- | --- | --- |
| 640x480 | `ov9281_640x480` | `config/ov9281_640x480.yaml` |
| 1024x768 | `ov9281_1024x768` | `config/ov9281_1024x768.yaml` |
| 1280x800 | `ov9281_1280x800` | `config/ov9281_1280x800.yaml` |

切换 profile 时必须同时传入匹配的 `image_width`、`image_height`、`camera_name` 和
`camera_info_url`。`framerate` 必须是该分辨率实际支持的值。

例如切换到 `1024x768 @ 30fps`：

```bash
roslaunch front_marker_tracking front_marker_tracking.launch \
  image_width:=1024 image_height:=768 framerate:=30.0 \
  camera_name:=ov9281_1024x768 \
  camera_info_url:=file:///home/vslam/seeker/src/front_marker_tracking/config/ov9281_1024x768.yaml
```

切换到 `1280x800 @ 30fps`：

```bash
roslaunch front_marker_tracking front_marker_tracking.launch \
  image_width:=1280 image_height:=800 framerate:=30.0 \
  camera_name:=ov9281_1280x800 \
  camera_info_url:=file:///home/vslam/seeker/src/front_marker_tracking/config/ov9281_1280x800.yaml
```

## Marker 配置

正式目标固定为：

- 字典：`DICT_5X5_1000`
- Marker ID：`33`
- 黑色码边长：`0.070m`

检测器会检查图像尺寸、CameraInfo 尺寸和 `usb_camera_optical_frame`。标定不匹配时不发布
公制位姿。

## 启动

加载构建后的工作空间：

```bash
source /opt/ros/noetic/setup.bash
source /home/vslam/seeker/devel/setup.bash
```

只启动相机和检测器：

```bash
roslaunch front_marker_tracking front_marker_detection.launch
```

启动相机、检测器和跟踪控制：

```bash
roslaunch front_marker_tracking front_marker_tracking.launch
```

完整跟踪入口默认保持：

```text
enable_setpoint_output=false
enable_yaw_control=false
publish_annotated=false
```

需要观察检测结果时可开启标注图：

```bash
roslaunch front_marker_tracking front_marker_tracking.launch publish_annotated:=true
```

## 主要话题

- `/usb_cam/image_raw`
- `/usb_cam/camera_info`
- `/front_marker_tracking/transform`
- `/front_marker_tracking/detector_status`
- `/front_marker_tracking/image_annotated`
- `/front_marker_tracking/marker_pose_camera`
- `/front_marker_tracking/position_error_camera`
- `/front_marker_tracking/yaw_error`
- `/front_marker_tracking/error_flu`
- `/front_marker_tracking/error_flu_filtered`
- `/front_marker_tracking/cmd_vel_camera`
- `/front_marker_tracking/cmd_vel_flu`
- `/front_marker_tracking/setpoint_preview`
- `/front_marker_tracking/status`

## 坐标和控制安全

相机必须正装朝前，画面上方对应机体上方。`config/front_usb_camera_aligned.yaml` 使用标准
optical 到 FLU 的轴变换，默认不应用相机光心到机体原点的平移：

```text
camera optical: x right, y down, z forward
FLU:            x forward, y left, z up
target_xyz_body: [0.8, 0.0, 0.0]
```

只有显式设置 `enable_setpoint_output:=true` 才创建
`/mavros/setpoint_raw/local` 发布者。非零输出仍要求 MAVROS 已连接、飞行器已解锁、模式为
`OFFBOARD`，并且视觉位姿和状态时间戳新鲜。本包不会自动解锁、切换模式或起飞。

## 输入检查

```bash
rostopic hz /usb_cam/image_raw
rostopic echo -n 1 /usb_cam/image_raw/header
rostopic echo -n 1 /usb_cam/camera_info
rostopic echo /front_marker_tracking/detector_status
rostopic hz /front_marker_tracking/transform
```
