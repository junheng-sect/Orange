# ov9281_usb_cam

SEEKER 上 OV9281 的 ROS Noetic MJPEG 相机层。包负责固定 UVC 设备、图像发布和
CameraInfo，不包含 Marker 检测或运动控制。

## 运行参数入口

日常运行参数全部在 `config/ov9281_common.yaml` 中修改（设备路径、分辨率、帧率、
采集/解码格式、坐标系、标定文件地址），保存后重启 launch 生效，无需改代码或
launch。

启动相机（板端完整命令）：

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
roslaunch ov9281_usb_cam ov9281.launch
```

首次构建前 `devel/setup.bash` 不存在属正常情况。

### 设备映射

当前相机在板端映射为 `/dev/video20`（同设备的第二个节点为 `/dev/video21`）。
配置不写 `/dev/videoN` 序号，而是用 USB `by-id` 路径锁定同一台相机：

```
/dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0
```

重新插拔或重启后可用以下命令核对映射是否仍一致：

```bash
ls -l /dev/v4l/by-id/
readlink -f /dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0
v4l2-ctl -d /dev/v4l/by-id/usb-DECXIN_CAMERA_DECXIN_CAMERA_01.00.00-video-index0 --list-formats-ext
```

`by-id` 路径由 USB 设备序列号生成，跨重启保持稳定；`/dev/videoN` 序号会漂移，
不要直接写进配置。

场景差异（raw 输出编码 `output_encoding`）维护在场景 YAML 中，launch 固定
"先公共、后场景"的加载顺序：

- `config/ov9281_rgb8.yaml` —— 普通相机查看，`rgb8`（默认入口）。
- `config/ov9281_mono8.yaml` —— 视觉跟踪，`mono8`（由
  `front_marker_tracking.launch` 自动选用）。

切换场景或配置文件：

```bash
roslaunch ov9281_usb_cam ov9281.launch scene_config:=$(rospack find ov9281_usb_cam)/config/ov9281_mono8.yaml
roslaunch ov9281_usb_cam ov9281.launch run_config:=/path/to/my_common.yaml scene_config:=/path/to/my_scene.yaml
```

`mono8` 模式直接复制 FFmpeg 解码帧的 8 位亮度平面；遇到不能直接复制的像素格式时
回退为 `AV_PIX_FMT_GRAY8` 转换。`/usb_cam/image_raw/compressed` 始终保持相机原始
MJPEG 直通，不受 raw 输出编码影响。

## 分辨率与标定

仓库保留 `640x480`（默认）、`1024x768` 和 `1280x800` 三套标定 YAML，各自只保存
标定数据。切换分辨率时在 `ov9281_common.yaml` 中同步修改 `image_width`、
`image_height`、`camera_name` 和 `camera_info_url` 四项。
`camera_info_url` 使用 `package://ov9281_usb_cam/config/…` 形式指向包内标定文件，
不依赖任何绝对路径；标定 YAML 中的 `camera_name` 应与运行参数 `camera_name`
保持一致。两者不一致时标定仍会照常加载，不会因此失败，只在加载时记录一条告警。

节点源码以官方 `usb_cam` 0.3.7 为基线，使用 `sws_getCachedContext()` 复用 FFmpeg
转换上下文；系统 `/opt/ros/noetic` 中的 apt 包不会被覆盖。rgb8 的颜色转换仍由 CPU
完成，mono8 的常见 planar YUV 路径不再生成 RGB24。
