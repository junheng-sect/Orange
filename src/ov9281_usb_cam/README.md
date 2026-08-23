# ov9281_usb_cam

SEEKER 上 OV9281 的 ROS Noetic MJPEG 相机层。包负责固定 UVC 设备、图像发布和
CameraInfo，不包含 Marker 检测或运动控制。

默认 profile 为 `640x480 @ 30fps`，使用 `config/ov9281_640x480.yaml`：

```bash
roslaunch ov9281_usb_cam ov9281.launch
```

仓库还保留 `1024x768` 和 `1280x800` 标定。切换时必须同时提供匹配的
`image_width`、`image_height`、`camera_name` 和 `camera_info_url`。

节点源码以官方 `usb_cam` 0.3.7 为基线，使用 `sws_getCachedContext()` 复用 FFmpeg
颜色转换上下文；系统 `/opt/ros/noetic` 中的 apt 包不会被覆盖。颜色转换仍由 CPU 完成。
