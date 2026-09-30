# barcode_reader_camera

相机读码 ROS Noetic 包，移植自 Drone-Vision `vision_core/src/vision_detect.cpp` 的 `decode_barcodes`（ZXing-C++ 3.0）。
流程与原实现一致：AprilTag 位姿 → 按标签物理尺寸投影 ROI → 透视矫正 / ROI / 1D 补救 → 全图金字塔 → 去畸变 → 换帧重试，
所有结果经 `barcode_result_fusion.h`（原样复制）按物理位置去重。策略名（`WARP-2d`、`ROI-raw`、`PFULL-1d-otsu` …）与
`vision_run.log` 相同，便于对照。串口读码器见同工作空间的 `barcode_reader_serial`。

## 看图工具 decode_image

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
rosrun barcode_reader_camera decode_image 图片.jpg --annotate /tmp/out.jpg
```

- 先用 `ov13855_apriltag_vision` 的检测器和 solvePnP 找 tag（tag36h11、长边 1280、选面积最大），找到就走标签 ROI，找不到只走全图金字塔。
- 内参：`--camera-yaml` 读 ROS camera_info YAML；`--focal-px F` 指定焦距；都不给时按手机等效 26 mm 估算（f = 0.6 × 对角线，主点在图像中心）。
- `--config yaml` 读取参数（默认值与 `config/barcode_reader_camera.yaml` 相同）；`--all-formats` 打开全部 8 种码制；
  `--free` 把 expected_codes 全置 0，完整跑一遍不提前结束；`--no-tag` 强制只走全图；`--verbose` 输出每个策略的日志。
- 每个码一行：`#序号 [码制] code=... raw_hex=... symbology=... center=(x,y) source=策略名`；标注图上序号与输出一致，
  青色框 = 标签 ROI，品红 = tag，绿色 = 已解出，红色 = ROI 内定位到但没解出。

## ROS 节点

```bash
roslaunch barcode_reader_camera bringup.launch        # 相机 + AprilTag + 读码节点，一条命令启动
rosrun barcode_reader_camera trigger_decode           # 触发一次读码（另开终端，可重复执行）
```

- `bringup.launch` include `ov13855_cam/ov13855.launch`、`ov13855_apriltag_vision.launch`、`barcode_reader_camera.launch`，
  参数 `camera_config` / `vision_config` / `config_file` / `require_tag` / `log_strategies` 透传。它会打开相机，与
  `front_marker_control/ov13855_marker_tracking.launch` 二选一；相机和 AprilTag 已经在跑时只启动 `barcode_reader_camera.launch`。
- `trigger_decode` 等 `~decode` 服务上线后调用一次，输出格式同 `decode_image`；链路还没就绪（`no_image` /
  `no_camera_info` / `no_tag`）时每 0.5 s 重试，直到 `--timeout`（默认 5 s）。退出码：0 读齐，1 未读齐或出错，2 服务不可用。
  也可以直接 `rosservice call /barcode_reader_camera/decode "{}"`。

- 订阅 `/ov13855/image_raw`、`/ov13855/camera_info`、`/front_marker/pose`。位姿沿用源图像时间戳，节点按时间戳精确配对，
  转回 rvec/tvec 投影 ROI。
- `~decode`（`DecodeBarcodes.srv`）：解码最新一帧；重试时等待之后的新帧。返回 `success / partial / error / elapsed_s /
  used_tag_roi / report / codes[]`，每个码是 `Barcode.msg`：`format, code, raw_hex, symbology_id, center_x/y, corners, source`。
- `~cancel`（`std_srvs/Trigger`）：中止正在进行的解码，`~decode` 返回 `cancelled`；空闲时 `success=false`。
- `~result`（`BarcodeArray.msg`）：每次 `~decode` 结束都发布；`~image_annotated` 有订阅者时发布标注图。
- `error`：`no_image`（没有 1 s 内的新图）、`no_camera_info`、`no_tag`（最新帧没有对应位姿）、`busy`、`disabled`、
  `no_formats`、`decode_fail`、`partial`（读到了但没达到 expected_codes）、`cancelled`。
- 参数：`~config_file`（默认 `config/barcode_reader_camera.yaml`）、`~require_tag`（默认 true，与原实现一样没有 tag 不解码；
  false 时没有 tag 就走全图）、`~max_frame_age_s`（1.0）、`~log_strategies`（false；true 时每个策略的日志打成 INFO）。

## 参数文件

`config/barcode_reader_camera.yaml`，结构同 Drone-Vision `config.json` 的 `tag` / `detector` / `barcode_decoder`，
默认值取自板端 `Drone-Vision/publish/config.json`：标签 0.10×0.15 m、tag 居中、roi_scale 1.25；启用 QR/DM/Code128，
预期数量 QR 1、DM 1、Code128 2；CLAHE 2.0/8；try_rotate；重试 2 次、间隔 100 ms；2D 没读齐时去畸变重扫。

## 与原实现的差异

- 输出从定长 C 缓冲改为码列表，新增 `raw_hex`（ZXing `bytes()`）、`symbology_id`、角点和命中策略，不再有 `truncated`。
- 没有 tag 或没有内参时跳过 ROI 和去畸变（原实现没有 tag 直接不解码；节点用 `require_tag` 保持该行为）。
- 取消在每次 ZXing 调用前和每个大步骤前检查（原实现只在重试间检查），`~cancel` 约 20 ms 内生效。
- 重试抓帧改为从 ROS 图像话题取新帧；1500 ms 总时长限制不变。
- `FUSION-CHECK` 不再把 detected 与 fusion 计数不一致记为 ERROR：same-symbol 候选进入 fusion 但不导出，二者本来就可能不同。

## 构建和测试

```bash
cd /home/orangepi/orange
catkin build barcode_reader_camera
catkin run_tests barcode_reader_camera --no-deps && catkin_test_results build/barcode_reader_camera
```

板端 ZXing 是只读构建（不能生成码），测试用的合成标签由 `test/synthetic_label.hpp` 生成：QR 点阵由 qrencode 预先导出，
Code128 用自带编码器，tag 用 `apriltag_to_image`。覆盖：Drone-Vision fusion 单测、tag ROI 读全且全图金字塔不运行、
无 tag 走金字塔、`--free` 跑全链路去重、未达预期报 partial、关闭的码制不扫描、重试取帧、取消、节点 `~decode` /
`no_tag` / `~cancel` / `~result`。DataMatrix 解码由实拍图验证。
