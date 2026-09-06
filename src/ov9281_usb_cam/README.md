# ov9281_usb_cam

SEEKER 上 OV9281 的 ROS Noetic MJPEG 相机层。包负责固定 UVC 设备、图像发布和
CameraInfo，不包含 Marker 检测或运动控制。

## 运行参数入口

日常运行参数全部在 `config/ov9281_common.yaml` 中修改（设备路径、分辨率、帧率、
采集/解码格式、坐标系、标定文件地址），保存后重启 launch 生效，无需改代码或
launch。

启动相机：

```bash
roslaunch ov9281_usb_cam ov9281.launch
```

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

节点源码以官方 `usb_cam` 0.3.7 为基线，使用 `sws_getCachedContext()` 复用 FFmpeg
转换上下文；系统 `/opt/ros/noetic` 中的 apt 包不会被覆盖。rgb8 的颜色转换仍由 CPU
完成，mono8 的常见 planar YUV 路径不再生成 RGB24。
