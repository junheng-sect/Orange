# ov9281_usb_cam

SEEKER 上 OV9281 的 ROS Noetic MJPEG 相机层。包负责固定 UVC 设备、图像发布和
CameraInfo，不包含 Marker 检测或运动控制。

默认 profile 为 `640x480 @ 30fps`，使用 `config/ov9281_640x480.yaml`：

```bash
roslaunch ov9281_usb_cam ov9281.launch
```

`output_encoding` 控制解码后的 raw 图像，允许 `rgb8`（默认，兼容既有订阅者）
和 `mono8`：

```bash
roslaunch ov9281_usb_cam ov9281.launch output_encoding:=mono8
```

mono8 模式直接复制 FFmpeg 解码帧的 8 位亮度平面；遇到不能直接复制的像素格式时
回退为 `AV_PIX_FMT_GRAY8` 转换。`/usb_cam/image_raw/compressed` 始终保持相机原始
MJPEG 直通，不受 raw 输出编码影响。

仓库还保留 `1024x768` 和 `1280x800` 标定。切换时必须同时提供匹配的
`image_width`、`image_height`、`camera_name` 和 `camera_info_url`。

节点源码以官方 `usb_cam` 0.3.7 为基线，使用 `sws_getCachedContext()` 复用 FFmpeg
转换上下文；系统 `/opt/ros/noetic` 中的 apt 包不会被覆盖。rgb8 的颜色转换仍由 CPU
完成，mono8 的常见 planar YUV 路径不再生成 RGB24。
