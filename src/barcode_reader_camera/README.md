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
- `--config yaml` 读取参数（默认值与 `config/barcode_reader_camera.yaml` 相同）；`--layout config/layouts/<名字>.yaml`
  叠加标签版式（含 tag 边长，不给时 tag 边长按 0.012 m）；`--all-formats` 打开全部 8 种码制；
  `--free` 把 expected_codes 全置 0，完整跑一遍不提前结束；`--no-tag` 强制只走全图；`--verbose` 输出每个策略的日志。
- 每个码一行：`#序号 [码制] code=... raw_hex=... symbology=... center=(x,y) source=策略名`；标注图上序号与输出一致，
  青色框 = 标签 ROI，品红 = tag，绿色 = 已解出，红色 = ROI 内定位到但没解出。

## ROS 节点

```bash
roslaunch barcode_reader_camera bringup.launch        # 相机 + AprilTag + 读码节点，一条命令启动
roslaunch barcode_reader_camera bringup.launch layout:=drone_vision_default   # 换一种标签版式
rosrun barcode_reader_camera trigger_decode           # 触发一次读码（另开终端，可重复执行）
```

- `bringup.launch` include `ov13855_cam/ov13855.launch`、`ov13855_apriltag_vision.launch`、`barcode_reader_camera.launch`，
  参数 `camera_config` / `vision_config` / `config_file` / `layout` / `layout_file` / `require_tag` / `log_strategies` 透传，
  同一个版式文件同时给视觉节点和读码节点。它会打开相机，与
  `front_marker_control/ov13855_marker_tracking.launch` 二选一；相机和 AprilTag 已经在跑时只启动 `barcode_reader_camera.launch`。
- `trigger_decode` 等 `~decode` 服务上线后调用一次，输出格式同 `decode_image`；链路还没就绪（`no_image` /
  `no_camera_info` / `no_tag`）时每 0.5 s 重试，直到 `--timeout`（默认 5 s）。退出码：0 读齐，1 未读齐或出错，2 服务不可用。
  也可以直接 `rosservice call /barcode_reader_camera/decode "{}"`。

- 订阅 `/ov13855/image_raw`、`/ov13855/camera_info`、`/front_marker/pose`。位姿沿用源图像时间戳，节点按时间戳精确配对，
  转回 rvec/tvec 投影 ROI。视觉节点要算 AprilTag，位姿比图像晚约 30–50 ms；`~decode` 先等最新帧的位姿（最多
  `~pose_wait_s`，收到更新帧的位姿说明这帧被视觉节点跳过，提前停止等待），再选最近一帧有位姿的图像（最多比最新帧早
  `~pose_fallback_s`），都没有才返回 `no_tag`。
- `~decode`（`DecodeBarcodes.srv`）：解码最新一帧；重试时等待之后的新帧。返回 `success / partial / error / elapsed_s /
  used_tag_roi / report / codes[] / header`（`header` 为所解图像的时间戳和坐标系，与 `~result` 相同；没取到图像时为空），每个码是 `Barcode.msg`：`format, code, raw_hex, symbology_id, center_x/y, corners, source`。
- `~cancel`（`std_srvs/Trigger`）：中止正在进行的解码，`~decode` 返回 `cancelled`；空闲时 `success=false`。
- `~result`（`BarcodeArray.msg`）：每次 `~decode` 结束都发布；`~image_annotated` 有订阅者时发布标注图。
- `error`：`no_image`（没有 1 s 内的新图）、`no_camera_info`、`no_tag`（等待后最近几帧都没有对应位姿）、`busy`、`disabled`、
  `no_formats`、`decode_fail`、`partial`（读到了但没达到 expected_codes）、`cancelled`。
  节点日志：成功以及 `decode_fail`、`partial` 打 INFO（定位到了但没读出，是正常结果）；`no_image`、`no_tag` 等链路问题打 WARN。
- 参数：`~config_file`（默认 `config/barcode_reader_camera.yaml`）、`~layout_file`（标签版式，launch 默认
  `config/layouts/qr_top_tag_bottom.yaml`；为空时用代码默认值并警告）、`~vision_tag_size_param`（默认
  `/ov13855_apriltag_vision_node/target_marker_size_m`，与版式 tag 边长不一致时每 10 s 警告一次，不阻断解码）、`~require_tag`（默认 true，与原实现一样没有 tag 不解码；
  false 时没有 tag 就走全图）、`~max_frame_age_s`（1.0）、`~pose_wait_s`（0.2）、`~pose_fallback_s`（0.2）、`~log_strategies`（false；true 时每个策略的日志打成 INFO）。

## 参数文件

`config/barcode_reader_camera.yaml`，结构同 Drone-Vision `config.json` 的 `tag` / `detector` / `barcode_decoder`，
默认值取自板端 `Drone-Vision/publish/config.json`：roi_scale 1.25；启用 QR/DM/Code128；CLAHE 2.0/8；try_rotate；
重试 2 次、间隔 100 ms；2D 没读齐时去畸变重扫。标签几何、码制开关和预期码数放在版式文件里。

## 标签版式

一种标签一个文件：`config/layouts/<名字>.yaml`，启动时 `layout:=<名字>` 选择。视觉节点（tag 边长 → 位姿尺度）和
读码节点（标签 ROI、formats、expected_codes）读同一个文件，换标签只换这个参数。

| 版式 | 内容 |
| --- | --- |
| `qr_top_tag_bottom`（默认） | 2026-10-08 现场标签：tag 43.7 mm，竖版 100×150 mm，QR 在上、tag 在下方居中，只开 QR、要 1 个 |
| `drone_vision_default` | Drone-Vision 原版：tag 12 mm，100×150 mm，tag 居中，QR×1 + DM×1 + Code128×2 |

```yaml
target_marker_size_m: 0.0437   # tag 黑框外沿边长（不含白边）；与视觉节点参数同名，加载时覆盖其默认值
label:
  width_meters: 0.10           # 标签纸宽，沿 tag 左右方向
  height_meters: 0.15          # 标签纸高，沿 tag 上下方向
  tag_position: custom         # center/top_left/top_right/bottom_left/bottom_right/custom
  tag_offset_x_meters: 0.0     # 仅 custom：tag 中心相对纸中心，x 向右
  tag_offset_y_meters: -0.0225 #             y 向 tag 正立的上方（印字方向）
formats: {QRCode: true, DataMatrix: false, Code128: false, ...}   # 标签上有哪些码制
expected_codes: {QRCode: 1, DataMatrix: 0, Code128: 0, ...}       # 每种要读到几个
```

- 标签上没有的码制要在 `formats` 里关掉：沿用原实现语义，启用但预期为 0 的码制按「顺带扫描」处理、永远不算读齐，
  解码会跑满 1.5 s 并返回 `partial`。

- 标签 ROI 只由 tag 位姿和版式推算，程序不检测标签纸边缘；版式填错不会漏读，但 ROI/透视矫正白跑，退到全图兜底。
- `tag_position` 的四个角表示 tag 中心正好落在纸的对应角上；tag 在纸内任意位置用 `custom` + 偏移。
- 新增版式：复制一个现有文件改名，量 tag 黑框边长、纸宽高、tag 中心到纸中心的偏移后填入，或者拍一张正视图，
  以 tag 位姿把纸的四角反投到 tag 平面算出来；用 `decode_image --layout ... --annotate` 看青色框是否罩住整张纸。
- 分开启动时（`ov13855_marker_tracking.launch` + `barcode_arrival_trigger/auto_read.launch`）两边要传同一个 `layout`。

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
