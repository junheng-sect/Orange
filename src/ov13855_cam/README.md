# ov13855_cam

ROS Noetic OV13855/RKISP 独立驱动包。传感器固定输出 4224×3136，ISP 同时出两路，各由一个节点发布：

| 节点 | 设备 | 输出 | 话题 | 用途 |
| --- | --- | --- | --- | --- |
| `ov13855_cam` | `/dev/video12` 自通路 | 1280×960 | `/ov13855/image_raw`、`/ov13855/camera_info` | AprilTag 跟踪（视觉节点一直订阅） |
| `ov13855_full_cam` | `/dev/video11` 主通路 | 4224×3136 | `/ov13855_full/image_raw`、`/ov13855_full/camera_info` | 读码（读码节点读码时才订阅） |

两路都是整幅 4224×3136 由 ISP 硬件缩放（crop 均为 (0,0)/4224×3136），视场相同；同一传感器帧两路时间戳差约 0.01 ms，
共用 `ov13855_optical_frame`。都是 NV12、15 FPS，直接把 Y 平面（亮度）作为 mono8 发布，UV 平面不读；
不翻转或缩放图像，曝光/增益由板端现有 ISP/3A 管理。不依赖 `ov9281_usb_cam`。

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
roslaunch ov13855_cam ov13855.launch              # 两路都起
roslaunch ov13855_cam ov13855.launch full:=false  # 只起 1280×960（只跟踪 tag、不读码）
```

参数入口：自通路 `config/ov13855.yaml`，主通路 `config/ov13855_full.yaml`；
可用 `config_file:=`、`full_config_file:=` 换成自定义文件。两路各自一个进程，一路重启不影响另一路。

图像只在有订阅者时才拷贝和发布；没有订阅者时仍从 ISP 取帧（保持出流），`camera_info` 每帧都发。
主通路平时没有订阅者，节点约占 4% CPU；读码节点订阅后约 0.1~0.3 s 收到第一帧（订阅握手 + 最多一帧间隔）。
图像经 image_transport 发布，另有 `/compressed` 等插件话题，只在有订阅者时编码；
电脑通过 `http://localhost:8765/` 的 OV13855 面板按需观看硬编码图传。
GStreamer appsink 和 ROS 发布队列均为 1；逐帧校验协商后的尺寸/格式/FPS，按实际行步长复制。

Y 平面当作 mono8 的前提是 ISP 输出全范围：colorimetry 第一位为 `1`（0~255）。颜色矩阵随分辨率而变
（1280×960、2560×1440 协商为 `1:3:5:1` 即 BT.709，4224×3136 为 `1:6:11:7` 即 BT.2020），只取亮度时不影响结果。
协商到限制范围（16~235）时节点报错退出，范围未知时告警一次。启动日志会打印设备、尺寸和实际 colorimetry。

管线不经 `videoconvert`（2026-10-14 去掉）：它会结合色度按颜色矩阵重算灰度，单线程处理，
2560×1440 时节点约占 0.9 核，4224×3136 时只能跑到 5.4 FPS；2560×1440 下输出与 Y 平面约 89% 像素不同，
平均差 1.9、最大差 17（同一帧离线复现）。去掉后板端实测（节点自身 CPU，无订阅 / 1 个订阅者）：
2560×1440 为 12% / 26%，4224×3136 为 15% / 63% 且稳定 15 FPS（当时无订阅也拷贝，现已跳过）。
帧时间戳由 GStreamer PTS 的帧龄映射至 ROS 时间；PTS 无效则告警并采用接收时间，
不宣称是硬件曝光时间。无帧 5 秒或采集错误会退出并释放设备。

## 标定来源与限制

| 文件 | 来源 |
| --- | --- |
| `config/ov13855_4224x3136.yaml` | 2026-10-15 主通路原生分辨率棋盘格标定（7x5 内角点、39.7 mm，`camera_calibration_pkg` 采集，`recalibrate.py --max-image-error 1.0` 离线重算）：k1、k2、k3 三项径向畸变，使用 23/27 张，RMS 0.59 px，fx/fy 标准差 ±3.8 px、cx/cy ±2.0 px（bootstrap fx ±7.5、cx/cy ±3 px）；角点最远到半对角线 0.85 处，画面四角畸变为外推 |
| `config/ov13855_1280x960.yaml` | 由上面按纯缩放换算（`camera_calibration_pkg/scale_calibration.py`），未重新标定 |
| `config/ov13855_2560x1440.yaml` | 2026-10-11 棋盘格标定（7x5 内角点、39.5 mm，`recalibrate.py` 离线重算）：k1、k2、k3 三项径向畸变，使用 25/29 张，RMS 0.32 px，fx/fy/cx/cy 标准差 ≤1.7 px；同日另一组独立标定各参数差 ≤3.5 px |

换算：K、P 的第一行乘 W1/W0、第二行乘 H1/H0，主点按像素中心 (c+0.5)·s−0.5，畸变系数不变；
前提是各分辨率都是同一画幅的整幅缩放（已在板端确认）。2560×1440 标定换算到 4224×3136 后 fx/fy=1.0007，
与 4224×3136 直接标定的 fx/fy=1.0003 一致，fx/fy/cx/cy 差 3~5 px，在标定误差内；2560×1440 的 fx/fy≈1.32
来自全幅 4:3 经 ISP 输出 16:9。替换前的两份换算文件备份在板端 `~/backup/ov13855_calib_scaled_from_2560_20261014/`。

节点不会自动缩放内参，标定尺寸与配置尺寸不符时报错退出。重新标定见 `camera_calibration_pkg` README
（`stream:=full` 标 4224×3136，再用 `scale_calibration.py` 换算 1280×960）。
2560×1440 文件保留作历史标定与交叉核对，当前没有配置使用它。

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
