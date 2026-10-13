# barcode_reader_camera 细节

使用方法见 [README.md](README.md)。这里是 ROS 接口全集、读码流程、记录文件字段和与原实现的差异，调试或改代码时看。

## 来源和流程

移植自 Drone-Vision `vision_core/src/vision_detect.cpp` 的 `decode_barcodes`（ZXing-C++ 3.0）。
第一帧流程与原实现一致：AprilTag 位姿 → 按标签物理尺寸投影 ROI → 透视矫正 / ROI / 1D 补救 → 全图金字塔 → 去畸变；
没读齐时换新帧重试（与原实现不同，见「换帧重试」）。所有结果经 `barcode_result_fusion.h`（原样复制）按物理位置去重。
策略名（`WARP-2d`、`ROI-raw`、`PFULL-1d-otsu` …）与 `vision_run.log` 相同，便于对照；重试帧上的策略名加 `RT-` 前缀。

## ROS 接口

- 订阅 `/ov13855/image_raw`、`/ov13855/camera_info`、`/front_marker/pose`。位姿沿用源图像时间戳，节点按时间戳精确配对，
  转回 rvec/tvec 投影 ROI。视觉节点要算 AprilTag，位姿比图像晚约 30–50 ms；`~decode` 先等最新帧的位姿（最多
  `~pose_wait_s`，收到更新帧的位姿说明这帧被视觉节点跳过，提前停止等待），再选最近一帧有位姿的图像（最多比最新帧早
  `~pose_fallback_s`），都没有才返回 `no_tag`。
- `~decode`（`DecodeBarcodes.srv`）：解码最新一帧，没读齐时换新帧重试。返回 `success / partial / error / elapsed_s /
  used_tag_roi / report / codes[] / header`（`header` 为结果所在那一帧的时间戳和坐标系：重试帧读出时是那一帧，否则是第一帧；
  与 `~result` 相同；没取到图像时为空），每个码是 `Barcode.msg`：`format, code, raw_hex, symbology_id, center_x/y, corners, source`，
  角点是该帧的像素坐标。`session_dir` 是本次运行的记录文件夹（没开记录时为空），`barcode_rc_trigger` 据此把
  `rc_read.csv` 写进同一个文件夹。
- `~cancel`（`std_srvs/Trigger`）：中止正在进行的解码，`~decode` 返回 `cancelled`；空闲时 `success=false`。
- `~result`（`BarcodeArray.msg`）：每次 `~decode` 结束都发布；`~image_annotated` 有订阅者时发布标注图。
- 日志级别：成功以及 `decode_fail`、`partial` 打 INFO（定位到了但没读出，是正常结果）；`no_image`、`no_tag` 等链路问题打 WARN。
- 私有参数（launch 已设置大部分）：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `~config_file` | `config/barcode_reader_camera.yaml` | 读码参数文件 |
| `~layout_file` | launch 默认 `config/layouts/qr_top_tag_bottom.yaml` | 标签版式；为空时用代码默认值并警告 |
| `~vision_tag_size_param` | `/ov13855_apriltag_vision_node/target_marker_size_m` | 与版式 tag 边长不一致时每 10 s 警告一次，不阻断解码 |
| `~require_tag` | `true` | 与原实现一样没有 tag 不解码；`false` 时没有 tag 走全图 |
| `~max_frame_age_s` | 1.0 | 最新图像超过这么久返回 `no_image` |
| `~pose_wait_s` | 0.2 | 等最新帧位姿的最长时间 |
| `~pose_fallback_s` | 0.2 | 没等到时，最多往前找多久有位姿的帧 |
| `~log_strategies` | `false` | `true` 时每个策略的日志打成 INFO |
| `~save_frames` / `~save_dir` / `~log_attempts` / `~save_retry_frames` | 见 README | 读码记录 |

- `trigger_decode`：等 `~decode` 服务上线后调用一次，输出格式同 `decode_image`；链路还没就绪（`no_image` /
  `no_camera_info` / `no_tag`）时每 0.5 s 重试，直到 `--timeout`（默认 5 s）。退出码：0 读齐，1 未读齐或出错，2 服务不可用。
  也可以直接 `rosservice call /barcode_reader_camera/decode "{}"`。

## 换帧重试

第一帧没读齐时，节点把之后的新帧一帧一帧送进解码，直到读齐或解码开始后 `barcode_decoder.retry.timeout_ms`（默认 2000 ms，
0 = 不重试）；最后一帧不打断，最多超出一帧的耗时：

- 每个重试帧用它自己的 tag 位姿（同时间戳的 `/front_marker/pose`）重新投影 ROI，只跑标签 ROI 策略（`RT-WARP-2d`、
  `RT-ROI-raw` …，按缺的码型补），不跑全图金字塔和去畸变。只有 QR/DM 时每帧约 30–50 ms，相机 15 fps 基本每帧都能试到；
  版式里有 Code128 时 1D 策略要多跑，每帧可能到几百 ms，2 s 内试的帧数少很多。
- 取帧规则：上一帧之后、位姿已到的最新一帧；位姿约晚 40 ms 到，所以重试帧通常是刚拍的那一帧。视觉节点没检测到 tag 的帧
  （太糊、出画）没有位姿，不试。
- 依据：2026-10-11 实飞读码失败几乎都是运动模糊（曝光约 30 ms 内机体晃动），模糊一阵一阵出现，换帧最有效。原实现的重试
  是把整帧缩到 512 扫两次（`RT-2d-small`），实飞 18 次失败一次也没读出；全图金字塔和去畸变每帧 100 ms 以上，实飞也从未读出。
  用实飞记录回放（每次只有主帧和 2 张重试帧）：36 次中成功从 16 次升到 25 次，救回的约 0.3 s 内读出。
- 读不出时 `~decode` 要等满 `timeout_ms` 才返回；`barcode_rc_trigger` 在这期间再拨杆会被忽略。

## 读码记录字段

写盘在后台线程进行，`~decode` 不等待写盘，读码耗时不变。写盘失败只告警，不影响读码。

- 会话文件夹：节点启动时定名 `<save_dir>/<YYYYmmdd_HHMMSS>_<layout_file 文件名>`（没有版式时为 `_default`，同一秒重名加 `_2`），第一次读码时才创建（`frame_recorder.cpp` 的 `sessionDir()`）。
- `attempts.csv`（`~log_attempts`，默认开）：时间、第一帧时间戳、结果、error、report、码内容、命中策略、总耗时/解码耗时、
  是否用了标签 ROI、tag 距离 `dist_m`、`z_m`、标签倾角 `tilt_deg`（标签法线与视线夹角）、ROI 外接框宽高、ROI 内平均亮度、
  ROI 内拉普拉斯方差（越大越清晰；以上都是第一帧的）、`frames`（共试了几帧：1 + 重试帧数）、
  `result_frame`（最后一个新码出自第几帧，0 = 第一帧，没读到码时为空）、记录目录名。
- 读码目录名 `<会话文件夹>/<YYYYmmdd_HHMMSS_mmm>_<ok|partial|错误名>/`。
- `meta.yaml`：结果（含 `frames`、`result_frame`）、码（含所在帧 `frame`）、第一帧的 tag 位姿（rvec/tvec，T_camera_tag）、
  距离和倾角、ROI 四角、定位到但没解出的框、亮度/清晰度、每个重试帧的序号/时间戳/位姿/文件名（没存图为空）、所用版式和参数文件。
- `decode.log` 与 `decode_image --verbose` 相同；`preview.jpg` 同 `~image_annotated`，重试帧读出时画在那一帧上。
- `no_tag` / `no_camera_info`（没进入解码）每 2 s 最多存一个目录，避免 `trigger_decode` 每 0.5 s 重试刷屏；CSV 照常每次一行。

## 参数文件

`config/barcode_reader_camera.yaml`，结构同 Drone-Vision `config.json` 的 `tag` / `detector` / `barcode_decoder`，
默认值取自板端 `Drone-Vision/publish/config.json`：roi_scale 1.25；CLAHE 2.0/8；try_rotate；2D 没读齐时去畸变重扫；
换帧重试到解码开始后 2 s（`retry.timeout_ms`；原实现的 `retry.count/interval_ms` 已删除，写了会报错）。
`detector` 只在 `decode_image` 自己检测 tag 时使用，ROS 节点用 `/front_marker/pose`。
标签几何、码制开关和预期码数放在版式文件里。

## 标签版式文件

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
  解码会重试到 `retry.timeout_ms` 并返回 `partial`。
- 标签 ROI 只由 tag 位姿和版式推算，程序不检测标签纸边缘；版式填错不会漏读，但 ROI/透视矫正白跑，退到全图兜底。
- `tag_position` 的四个角表示 tag 中心正好落在纸的对应角上；tag 在纸内任意位置用 `custom` + 偏移。
- 新增版式：复制一个现有文件改名，量 tag 黑框边长、纸宽高、tag 中心到纸中心的偏移后填入，或者拍一张正视图，
  以 tag 位姿把纸的四角反投到 tag 平面算出来；用 `decode_image --layout ... --annotate` 看青色框是否罩住整张纸。

## 与原实现的差异

- 输出从定长 C 缓冲改为码列表，新增 `raw_hex`（ZXing `bytes()`）、`symbology_id`、角点和命中策略，不再有 `truncated`。
- 没有 tag 或没有内参时跳过 ROI 和去畸变（原实现没有 tag 直接不解码；节点用 `require_tag` 保持该行为）。
- 取消在每次 ZXing 调用前和每个大步骤前检查（原实现只在重试间检查），`~cancel` 约 20 ms 内生效。
- 换帧重试改为：从 ROS 图像话题取有位姿的新帧，每帧用自己的位姿只跑标签 ROI，直到 `retry.timeout_ms`。
  原实现固定重试 2 帧、整帧缩到 512 扫描，1500 ms 总时长。
- `FUSION-CHECK` 不再把 detected 与 fusion 计数不一致记为 ERROR：same-symbol 候选进入 fusion 但不导出，二者本来就可能不同。
- WARP 矫正图改为标签正立：原实现把 ROI 的左下角映射到矫正图左上角，矫正图上下镜像（ZXing 对 QR/DM 会尝试镜像，
  实测解码结果和耗时不受影响）。坐标经 H⁻¹ 映射回原图，输出不变。
- `decode_image` 用 `ov13855_apriltag_vision` 的检测器和 solvePnP 找 tag（tag36h11、长边 1280、选面积最大）；不做换帧重试，单帧跑完整流程。

## 测试覆盖

板端 ZXing 是只读构建（不能生成码），测试用的合成标签由 `test/synthetic_label.hpp` 生成：QR 点阵由 qrencode 预先导出，
Code128 用自带编码器，tag 用 `apriltag_to_image`。覆盖：Drone-Vision fusion 单测、tag ROI 读全且全图金字塔不运行、
无 tag 走金字塔、`--free` 跑全链路去重、未达预期报 partial、关闭的码制不扫描、重试取帧、取消、读码记录、
节点 `~decode` / `no_tag` / `~cancel` / `~result`。DataMatrix 解码由实拍图验证。
