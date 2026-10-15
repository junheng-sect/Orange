# barcode_reader_camera

相机读码：用 AprilTag 位姿框出标签区域，在 OV13855 全分辨率（4224×3136）图像上解 QR / DataMatrix / Code128。
全分辨率图只在读码时订阅（相机主通路 `/ov13855_full/image_raw`），tag 位姿来自自通路 1280×960，见 `ov13855_cam` README。
提供 `~decode` 服务，平时由 `barcode_rc_trigger`（拨 CH9 手动读）或 `barcode_arrival_trigger`（到位自动读）调用。
ROS 接口全集、读码流程、与 Drone-Vision 原实现的差异见 [DETAILS.md](DETAILS.md)。

## 快速开始

先 `source /opt/ros/noetic/setup.bash && source /home/orangepi/orange/devel/setup.bash`。

| 场景 | 命令 |
| --- | --- |
| 拨 CH9 手动读码（最常用） | `roslaunch barcode_rc_trigger manual_read.launch`，见 `barcode_rc_trigger` README |
| 地面单独测：相机 + AprilTag + 读码一起起 | `roslaunch barcode_reader_camera bringup.launch` |
| 相机和 AprilTag 已在跑，只加读码节点 | `roslaunch barcode_reader_camera barcode_reader_camera.launch` |
| 命令行触发一次读码 | `rosrun barcode_reader_camera trigger_decode` |
| 离线看一张图 | `rosrun barcode_reader_camera decode_image 图片.jpg --annotate /tmp/out.jpg` |

`bringup.launch` 会打开相机，和 `ov13855_marker_tracking.launch`、带相机的 fusion 二选一。

忘了参数怎么填：

```bash
roslaunch --ros-args barcode_reader_camera bringup.launch                  # 列出每个参数的默认值和可选值
roslaunch --ros-args barcode_reader_camera barcode_reader_camera.launch
```

## launch 参数

| 参数 | 默认 | 可选值 |
| --- | --- | --- |
| `layout` | `qr_top_tag_bottom` | 见「标签版式」 |
| `save_frames` | `off` | `off` 不存图 / `failed` 只存失败 / `all` 都存，见「读码记录」 |
| `save_dir` | `~/test_data/barcode` | 记录根目录，每次启动建一个子文件夹，见「读码记录」 |
| `require_tag` | `true` | `true` 看不到 tag 不解码（返回 `no_tag`）/ `false` 看不到 tag 时扫全图 |
| `log_strategies` | `false` | `true` 每个解码策略的日志打成 INFO（调试用） |
| `output` | `screen` | `screen` 日志打到终端 / `log` INFO 只进 `~/.ros/log`，WARN/ERROR 仍打到终端 |
| `config_file` | `config/barcode_reader_camera.yaml` | 读码参数文件，见「参数文件」 |
| `save_retry_frames` | 2 | 每次最多存几张重试帧原图，`-1` 全存（只有 `barcode_reader_camera.launch` 有） |
| `log_attempts` | `true` | `true` 每次读码在 `attempts.csv` 记一行 / `false`（只有 `barcode_reader_camera.launch` 有） |
| `camera_config` / `vision_config` | 各包默认 YAML | 相机、AprilTag 参数文件（只有 `bringup.launch` 有） |

`manual_read.launch`、`auto_read.launch` 只透传其中的 `layout`、`save_frames`、`save_dir`。

## 标签版式

一种标签一个文件 `config/layouts/<名字>.yaml`，用 `layout:=<名字>` 选。视觉节点（tag 边长）和读码节点（标签区域、码制、要读几个）
读同一个文件，换标签只改这个参数。

| `layout` | 标签 |
| --- | --- |
| `qr_top_tag_bottom`（默认） | 2026-10-08 现场标签：tag 43.7 mm，竖版 100×150 mm，QR 在上、tag 在下，要读 1 个 QR |
| `dm_top_tag_bottom` | 2026-10-11 现场标签：同上，QR 换成 DataMatrix，要读 1 个 DM |
| `drone_vision_default` | Drone-Vision 原版：tag 12 mm，100×150 mm，tag 居中，QR×1 + DM×1 + Code128×2 |

新增标签：复制一个现有文件改名再量好填入，文件格式见 [DETAILS.md](DETAILS.md) 的「标签版式文件」。

## 读码结果

读码过程：订阅全分辨率图，约 0.2 s 后收到第一帧；取位姿已到的最新一帧，按版式框出标签区域解码；没读齐就换新帧重试，
每帧用自己的位姿只扫标签区域，直到读齐或解码开始后 2 s（参数文件里的 `retry.timeout_ms`），结束后退订。
所以第一帧就读出时约 0.2~0.3 s（主要是等第一帧），读不出时约 2.3 s。收到第一帧后 0.2 s 内的帧都没有 tag 位姿时返回 `no_tag`（共约 0.4~0.5 s），不重试。

`~decode` 返回的 `error`：

| `error` | 含义 |
| --- | --- |
| 空 | 读齐 |
| `partial` | 读到了，但没达到版式要求的个数 |
| `decode_fail` | 看到标签了，没读出码 |
| `no_tag` | 最近的帧都没有 tag 位姿（看不到 tag） |
| `no_image` | 订阅后 1 s 内没收到全分辨率图像（主通路节点 `ov13855_full_cam` 没在跑） |
| `no_camera_info` | 没收到相机内参 |
| `busy` | 上一次读码还没结束 |
| `cancelled` | 被 `~cancel` 中止 |
| `disabled` / `no_formats` | 参数文件里关掉了读码 / 一种码制都没开 |

## 读码记录

每次启动读码节点建一个文件夹 `~/test_data/barcode/<启动时间>_<版式>/`（例如 `20261013_150000_qr_top_tag_bottom/`），
这次启动的记录都在里面；启动后一次都没读码就不建。`~decode` 的返回值 `session_dir` 就是这个文件夹。

```
~/test_data/barcode/20261013_150000_qr_top_tag_bottom/
  attempts.csv                       # 默认就有：每次读码一行
  rc_read.csv                        # 用 manual_read 拨杆时才有：每次拨杆一行（tag 位置 + 结果），见 barcode_rc_trigger
  20261013_150012_345_ok/            # save_frames:=failed 或 all 时：一次读码一个目录
  20261013_150015_901_decode_fail/
```

- `attempts.csv`：结果、码、耗时、tag 距离和倾角、标签区域亮度和清晰度、试了几帧，最后一列是对应的读码目录名；不存图。

每个读码目录里（全分辨率 PNG 约 5 MB 一张：第一帧读出约 5 MB，重试到超时约 20 MB）：

- `frame.png`：第一帧原图（无损 PNG）。
- `retry_N.png`：第 N 个重试帧，只存前 `save_retry_frames` 张、读出码的那一帧和最后一帧。
- `preview.jpg`：缩小的标注图（JPEG）。
- `meta.yaml`：结果、码、tag 位姿、标签区域四角、每个重试帧的时间戳和位姿；`camera.yaml`：相机内参；`decode.log`：每个解码策略的日志。

图片格式（PNG / JPEG）写死在 `src/frame_recorder.cpp`，不能用参数改。

复现某一次读码：

```bash
cd ~/test_data/barcode/<启动文件夹>/<读码目录>
rosrun barcode_reader_camera decode_image frame.png --camera-yaml camera.yaml --pose meta.yaml \
  --layout <meta.yaml 里的 layout_file> --config <meta.yaml 里的 config_file> --verbose --annotate /tmp/replay.jpg
```

把 `frame.png` 换成 `retry_N.png` 复现某个重试帧（自动用该帧的位姿）。

## 离线看图 decode_image

```bash
rosrun barcode_reader_camera decode_image 图片.jpg --annotate /tmp/out.jpg
```

| 选项 | 作用 |
| --- | --- |
| `--annotate 输出.jpg` | 画标注图：青框 = 标签区域，品红 = tag，绿 = 读出，红 = 定位到但没读出 |
| `--layout config/layouts/<名字>.yaml` | 标签版式（不给时 tag 边长按 12 mm） |
| `--camera-yaml camera.yaml` / `--focal-px F` | 相机内参 / 只给焦距；都不给时按手机 26 mm 等效焦距估算 |
| `--pose meta.yaml` | 用节点记录里的 tag 位姿，不自己检测 tag |
| `--config yaml` | 读码参数文件（默认值同 `config/barcode_reader_camera.yaml`） |
| `--verbose` | 打印每个解码策略的日志 |
| `--all-formats` / `--free` / `--no-tag` | 打开全部 8 种码制 / 不提前结束、完整跑一遍 / 不找 tag、只扫全图 |

每个码输出一行：`#序号 [码制] code=... raw_hex=... symbology=... center=(x,y) source=策略名`，序号和标注图一致。

## 注意

- 标签上没有的码制要在版式的 `formats` 里关掉：启用但要求 0 个的码制永远不算读齐，每次都会重试满 2 s 并返回 `partial`。
- 分开启动（fusion + `manual_read.launch`，或 `ov13855_marker_tracking.launch` + `auto_read.launch`）时两边的 `layout` 要一致；
  tag 边长不一致时读码节点每 10 s 警告一次，位置也不准。
- `trigger_decode` 在链路没就绪（`no_image` / `no_camera_info` / `no_tag`）时每 0.5 s 重试，直到 `--timeout`（默认 5 s）。

## 参数文件

`config/barcode_reader_camera.yaml`，改完重启节点生效。常改的：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `barcode_decoder.retry.timeout_ms` | 2000 | 没读齐时换帧重试到解码开始后多久，0 = 不重试 |
| `tag.roi_scale` | 1.25 | 标签区域 = 标签尺寸 × roi_scale |
| `barcode_decoder.pyramid.with_tag` | `false` | 有 tag ROI 时第一帧是否再跑整图金字塔（1280/1920/原图）；没有 tag 时总是跑。ROI 切不准时改回 `true` |
| `barcode_decoder.undistort.enabled` | `false` | 2D 码没读齐时整幅去畸变再扫一次（有无 tag 都适用）；4224×3136 下每次约 370 ms，ROI 切不准时与上一项一起改回 `true` |

其余参数见文件内注释和 [DETAILS.md](DETAILS.md)。

## 构建和测试

```bash
cd /home/orangepi/orange
catkin build barcode_reader_camera
catkin run_tests barcode_reader_camera --no-deps && catkin_test_results build/barcode_reader_camera
```
