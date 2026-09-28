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
GStreamer appsink 和 ROS 发布队列均为 1；逐帧校验协商后的尺寸/格式/FPS，按实际行步长复制。
帧时间戳由 GStreamer PTS 的帧龄映射至 ROS 时间；PTS 无效则告警并采用接收时间，
不宣称是硬件曝光时间。无帧 5 秒或采集错误会退出并释放设备。

## 标定来源与限制

`config/ov13855_2560x1440.yaml` 复制自板端
`/home/orangepi/Drone-Vision/vision_core/config.json` 的内参和畸变。
原 JSON 没有标定分辨率字段；按用户要求，暂认为与其采集配置 2560×1440 对应。
这是配置假设，尚不能作为真实尺度精度验证。变更分辨率必须提供对应标定，不自动缩放内参。

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
