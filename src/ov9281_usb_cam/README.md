# ov9281_usb_cam

SEEKER 上 OV9281 的 ROS Noetic MJPEG 输入节点。源码以官方 `usb_cam` 0.3.7 为基线，
保留原 BSD 许可证；包名和可执行文件名独立，不覆盖 `/opt/ros/noetic` 中的 apt 包。

与上游 0.3.7 相比，本包只修改 FFmpeg 的颜色转换上下文生命周期：

- 使用 `sws_getCachedContext()` 复用相同输入格式和尺寸的 `SwsContext`；
- 节点退出时统一释放上下文；
- 不修改 FFmpeg 全局日志级别。

因此 `yuv422p -> rgb24` 缺少硬件加速的提示最多应在转换上下文首次创建时出现一次，
不会再随每帧重复。颜色转换本身仍由 libswscale 在 CPU 上完成。

正式入口由 `front_marker_tracking` 的 launch 提供，本包不安装额外 launch。
