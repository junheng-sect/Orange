# barcode_rc_trigger

拨一下遥控器 CH9，读一次码。终端打印一块汇总，结果和 tag 位置记进 CSV，用来测在什么位置能读成功。
本包只负责触发，真正读码的是 `barcode_reader_camera`（相机）和 `barcode_reader_serial`（串口读码器）。

## 快速开始

日常配合 fusion 使用（MAVROS 常驻，由 manual_read 启动 OV13855 和 AprilTag）：

```bash
# 终端 1
cd /home/orangepi/mid360s_ws/scripts && ./fusion.sh
# 终端 2
source /opt/ros/noetic/setup.bash && source /home/orangepi/orange/devel/setup.bash
source /home/orangepi/mid360s_ws/scripts/ros_net.sh
roslaunch barcode_rc_trigger manual_read.launch camera:=true
```

单独做相机读码地面测试时，板端使用常驻 MAVROS，运行上面的终端 2 命令即可。

自动移到码前、再手动拨杆读码时，先按上述步骤启动 fusion 和 manual_read。
再开一个终端（同样先 source、export ROS_IP），起相机 TF 和控制：

```bash
roslaunch front_marker_control ov13855_marker_tracking.launch camera:=false   # 复用 manual_read camera:=true 启动的相机和 AprilTag
```

飞机切 OFFBOARD 后自动移到 tag 正前方（相机到 tag 约 0.62 m），到位后拨 CH9。汇总标题会带上当时的控制模式
（如 `pulse#3 mode=POS_HOLD`），`rc_read.csv` 的 `control_mode` 列同样记录，方便只统计到位后的读码。
全自动（到位就读）见 `barcode_arrival_trigger`。

忘了参数怎么填：

```bash
roslaunch --ros-args barcode_rc_trigger manual_read.launch     # 列出每个参数的默认值和可选值
```

## manual_read.launch 参数

| 参数 | 默认 | 可选值 |
| --- | --- | --- |
| `mode` | `camera` | `camera` 只相机 / `serial` 只串口读码器 / `both` 先相机后读码器，两个都读 |
| `layout` | `qr_top_tag_bottom` | `qr_top_tag_bottom` QR 标签 / `dm_top_tag_bottom` DataMatrix 标签 / `drone_vision_default` 原版（需 `camera:=true`） |
| `camera` | `false` | `false` 复用已有相机 / `true` 本 launch 起相机和 AprilTag |
| `save_frames` | `off` | `off` 不存图 / `failed` 只存失败 / `all` 都存 |
| `save_dir` | `~/test_data/barcode` | 记录根目录，每次启动建一个子文件夹 |
| `quiet` | `true` | `true` 终端只留拨杆汇总和告警 / `false` 所有节点日志都打印 |
| `config_file` | `config/barcode_rc_trigger.yaml` | 触发节点参数文件（CH9 阈值、CSV 目录），见「参数文件」 |

例：DataMatrix 标签，只存读失败的图：

```bash
roslaunch barcode_rc_trigger manual_read.launch camera:=true layout:=dm_top_tag_bottom save_frames:=failed
```

## 结果在哪看

每次启动 `manual_read` 的结果都在一个文件夹里：`~/test_data/barcode/<启动时间>_<版式>/`
（例如 `20261013_150000_qr_top_tag_bottom/`），第一次拨杆时才创建。

| 看什么 | 在哪 |
| --- | --- |
| 每次拨杆的汇总 | 终端 |
| 每次拨杆一行（tag 位置 + 结果） | 启动文件夹里的 `rc_read.csv` |
| 相机每次读码一行（耗时、清晰度、试了几帧） | 启动文件夹里的 `attempts.csv` |
| 原图、重试帧、位姿、策略日志 | 启动文件夹里的 `<时间>_<结果>/`，`save_frames` 不是 `off` 时才有 |

只用串口读码器（`mode:=serial`）时没有相机记录，`rc_read.csv` 放在 `~/test_data/barcode/<启动时间>_serial/`。

终端汇总（`mode:=camera` 时只有 `camera` 段，含 serial 时后面多一段 `serial`；控制节点在跑时标题带 `mode=<控制模式>`）：

```
pulse#1 mode=POS_HOLD
tag:
  source: img
  id: 0
  x: -0.094
  y: -0.063
  z: 0.205
  dist: 0.234
  tilt_deg: 15.8
camera:
  ok: true
  report: "QR=1/1"
  codes:
    - "QRCode:12345678"
  ms: 26
---
```

- `tag`：tag 中心在相机光学系下的位置（x 右、y 下、z 前，m），`dist` 是直线距离，`tilt_deg` 是标签倾角（正对为 0）；
  拨杆时看不到 tag 为 `tag: none`。
- `camera.error` 常见值：`no_tag` 看不到 tag、`decode_fail` 看到了没读出、`no_image` 没有相机图像、`unavailable` 读码节点没起来；
  全部取值见 `barcode_reader_camera` README「读码结果」。
- `ms`：读码耗时。相机第一帧没读出会换帧重试，读不出时约 2 s。

## 注意

- 拨杆判定：CH9 先低于 1600 才布防，再高于 1700 并保持 0.05 s 触发一次；按住不放不会重复触发，回到低位才能再触发。
  第一次用先 `rostopic echo -n1 /mavros/rc/in`，看 `channels[8]` 的高低位是否在 1600 和 1700 两侧。
- 上一次读码还没结束（相机读不出时约 2 s）再拨杆会被忽略，日志提示 `pulse#N ignored`。
- `camera:=false` 复用已启动的任务相机和 AprilTag，两个入口使用相同的 layout。
- `camera:=true` 由 manual_read 管理相机；其他相机任务需要先结束或显式复用已有相机。
- `mode` 含 `serial` 时不要同时运行 DroneVision（共用 `/dev/ttyS1`）。
- 板端 `mavros/launch/px4.launch` 默认值已改成 `fcu_url` `/dev/ttyS3:921600`、`fcu_protocol` v1.0（原文件备份为 `px4.launch.orig`）；
  apt 升级 mavros 会覆盖这个改动，届时显式传 `fcu_url:=/dev/ttyS3:921600 fcu_protocol:=v1.0`。

---

## 细节

### 只起触发节点

`barcode_rc_trigger.launch` 只起本节点，读码节点另外启动（相机见 `barcode_reader_camera`，串口见 `barcode_reader_serial`）：

```bash
roslaunch barcode_rc_trigger barcode_rc_trigger.launch             # mode 默认 camera
roslaunch barcode_rc_trigger barcode_rc_trigger.launch mode:=both
```

### 行为

- `both` 先相机后读码器，两个都读（不是失败才用读码器），同一次拨杆拿两份结果对比。先相机是为了不把读码器的补光/瞄准光拍进图里。
- 某个读码服务 `service_wait_s` 内不在线就跳过它，另一个照常读；这一段写 `ok: false`、`error: unavailable`，整块汇总打 WARN。
  读到、读不到都打 INFO（读不到是测距实验的正常结果）。
- `tag.source`：`img` 是相机读码所用那一帧的位姿（重试帧读出时是那一帧；读码图是 4224×3136 主通路，位姿取自通路同一传感器帧，时间戳差 10 ms 内最近的一帧）；相机没取到图或只跑 serial 时用拨杆时刻最新一帧，记为 `pulse`。
  both 模式下读码器在相机之后才读，期间若在移动，位置以相机那一帧为准。
- 每次开始读码发布 `~pulse`（`std_msgs/UInt32`，拨杆序号；被忽略的拨杆也占号但不发布），用来在 bag 里对齐；
  详细结果看两个读码节点各自的 `~result`。
- CH9 不在 800~2200（通道缺失或未使用）或超过 `rc_timeout_s` 收不到时视为 RC 无效：撤防，必须重新见到低位。
  节点启动或 RC 重连时开关已在高位不会触发。
- CSV 每次启动一个文件，第一次拨杆时才创建：位置取相机 `~decode` 返回的 `session_dir`（相机读码节点这次启动的文件夹），
  没有时用 `csv_dir/<启动时间>_<mode>/`。列：
  `time,pulse,control_mode,tag_src,marker_id,x,y,z,dist,tilt_deg,camera_ok,camera_error,camera_report,camera_codes,camera_ms,serial_ok,serial_error,serial_code,serial_ms`；
  没调用的那一路留空，服务不在线时 `*_ok=0`、`*_error=unavailable`。

### 参数文件

`config/barcode_rc_trigger.yaml`，改完重启节点生效。`mode` 是 launch 参数；串口读码器的两项在 `config/serial.yaml`，
只在 `mode` 为 `serial` / `both` 时加载，所以 camera 模式下终端里没有串口相关内容。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `rc_topic` | `/mavros/rc/in` | RC 话题（`mavros_msgs/RCIn`） |
| `rc_channel` | 9 | 1-based 通道号 |
| `arm_below` / `trigger_above` | 1600 / 1700 | 布防 / 触发阈值（us） |
| `hold_s` | 0.05 | 高位保持多久才算一次拨杆 |
| `rc_timeout_s` | 1.0 | 超过这么久没有有效 CH9 视为 RC 丢失 |
| `camera_service` | `/barcode_reader_camera/decode` | 相机读码服务 |
| `service_wait_s` | 0.5 | 等读码服务上线的最长时间 |
| `transform_topic` | `/ov13855_apriltag_vision/debug/homogeneous_transform` | tag 位置来源，留空不记录 |
| `control_mode_topic` | `/front_marker_control/control_mode` | 控制模式来源（latch），记进标题和 `control_mode` 列；控制节点没在跑时为空；留空不订阅 |
| `tag_timeout_s` | 0.5 | 拨杆时刻最新 tag 的最大年龄 |
| `csv_dir` | `~/test_data/barcode` | 相机没给启动文件夹（只跑 serial）时 `rc_read.csv` 的根目录；留空不写 CSV |
| `serial_service`（`serial.yaml`） | `/barcode_reader_serial/read` | 串口读码服务 |
| `serial_timeout_s`（`serial.yaml`） | 0.0 | 传给 `~read` 的超时，<=0 用读码器节点的 `read_timeout_ms` |

### 构建和测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build barcode_rc_trigger
catkin run_tests barcode_rc_trigger --no-deps && catkin_test_results build/barcode_rc_trigger
```

测试不接触飞控和读码硬件：`test_policy.py` 测触发状态机（上电高位、去抖、按住、重新布防、中间区、RC 丢失），
`test_tag_record.py` 测 tag 位置换算和 CSV 行，`test_runner.py` 测调用顺序、单个服务失败不影响另一个、汇总格式和日志级别，
`node.test` 用假 RC 话题和假读码服务跑完整节点（触发、忙时忽略、RC 丢失撤防、服务离线）。
