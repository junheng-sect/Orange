# ov13855_cam

ROS Noetic OV13855/RKISP 独立驱动包。默认 `/dev/video11`，NV12 2560×1440、15 FPS，
经 GStreamer 转 mono8；不翻转或缩放图像，曝光/增益由板端现有 ISP/3A 管理。
不依赖 `ov9281_usb_cam`。

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
roslaunch ov13855_cam ov13855.launch
```

日常参数入口为 `config/ov13855.yaml`。自定义配置：
`roslaunch ov13855_cam ov13855.launch config_file:=/absolute/path/camera.yaml`。
输出 `/ov13855/image_raw`、`/ov13855/camera_info`，共用时间戳和 `ov13855_optical_frame`。
图像经 image_transport 发布，另有 `/ov13855/image_raw/compressed` 等插件话题，只在有订阅者时编码；
远程查看优先用视觉包的缩小预览 `/ov13855_apriltag_vision/image_preview/compressed`。
GStreamer appsink 和 ROS 发布队列均为 1；逐帧校验协商后的尺寸/格式/FPS，按实际行步长复制。
帧时间戳由 GStreamer PTS 的帧龄映射至 ROS 时间；PTS 无效则告警并采用接收时间，
不宣称是硬件曝光时间。无帧 5 秒或采集错误会退出并释放设备。

## 标定来源与限制

`config/ov13855_2560x1440.yaml` 为 2026-10-11 棋盘格标定（7x5 内角点、39.5 mm，`camera_calibration_pkg`
采集，`recalibrate.py` 离线重算）：k1、k2、k3 三项径向畸变，使用 25/29 张，RMS 0.32 px，
fx/fy/cx/cy 标准差 ≤1.7 px；同日另一组独立标定各参数差 ≤3.5 px。fx/fy≈1.32 与全幅 4:3 经 ISP 输出 16:9 一致。
此前为 `Drone-Vision/vision_core/config.json` 的 1920×1080 标定按 4/3 缩放的参考值，见 git 历史；
与新标定相比 cy 差约 8 px、边角畸变差异明显。变更分辨率必须重新标定，节点不会自动缩放内参。

板端构建：

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build ov13855_cam ov13855_apriltag_vision
```

异常路径测试（不打开真实相机）：

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin run_tests ov13855_cam --no-deps
```
