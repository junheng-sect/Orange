# barcode_arrival_trigger

在当前 Orange 检出中执行 `export ORANGE_ROOT="$(git rev-parse --show-toplevel)"`，下文命令以此定位仓库。

到位自动读码：`front_marker_control` 进入 `POS_HOLD`（到位定点）时自动读一次码，发布 `~result`，日志里打一块汇总，
结果和 tag 位置记进 `arrival_read.csv`。
本包只负责触发和发布；到位判定、失败重试、巡检航点调度都不在这里。手动拨 CH9 读码是 `barcode_rc_trigger`，两者互不干扰。

## 快速开始

只支持 OV13855 链路。实飞配合 fusion（MAVROS 常驻，fusion 启动定位），三个终端按顺序启动：

```bash
# 终端 1：fusion
cd "$ORANGE_ROOT"/scripts && ./fusion.sh
# 终端 2、3 先执行：
source /opt/ros/noetic/setup.bash && source "$ORANGE_ROOT"/devel/setup.bash && source "$ORANGE_ROOT"/scripts/ros_net.sh
# 终端 2：相机、AprilTag、TF 和到位控制
roslaunch front_marker_control ov13855_marker_tracking.launch
# 终端 3：相机读码节点 + 到位触发节点
roslaunch barcode_arrival_trigger auto_read.launch
```

飞机切 OFFBOARD 后由控制节点移到码前（tag 正前方机体 0.8 m，相机到 tag 约 0.62 m），进入 `POS_HOLD` 就自动读一次。

忘了参数怎么填：`roslaunch --ros-args barcode_arrival_trigger auto_read.launch`

## auto_read.launch 参数

| 参数 | 默认 | 可选值 |
| --- | --- | --- |
| `mode` | `camera` | `camera` 只相机 / `serial` 只串口读码器 / `both` 先相机后读码器，两个都读 |
| `layout` | `qr_top_tag_bottom` | `qr_top_tag_bottom` / `dm_top_tag_bottom` / `drone_vision_default`，必须与 AprilTag 那边相同 |
| `save_frames` | `failed` | `failed` 只存失败（到位读码很少失败，每次失败都留图）/ `all` 都存 / `off` 不存图 |
| `save_dir` | `~/test_data/barcode` | 记录根目录，每次启动建一个子文件夹 |
| `config_file` | `config/barcode_arrival_trigger.yaml` | 到位触发节点参数文件，见「参数文件」 |

`mode` 含 `serial` 时才启动串口读码器节点。版式说明见 `barcode_reader_camera` README「标签版式」。

## 结果在哪看

每次启动的结果都在一个文件夹里：`~/test_data/barcode/<启动时间>_<版式>/`，第一次读码时才创建。

| 看什么 | 在哪 |
| --- | --- |
| 每次到位的汇总 | 终端，格式同 `barcode_rc_trigger`，标题为 `arrival#N marker=M`，含 tag 段 |
| 每次到位一行（tag 位置 + 结果 + 读码中是否退出 POS_HOLD） | 启动文件夹里的 `arrival_read.csv` |
| 相机每次读码一行（耗时、清晰度、试了几帧） | 启动文件夹里的 `attempts.csv` |
| 原图、重试帧、位姿、策略日志 | 启动文件夹里的 `<时间>_<结果>/`；默认只有读失败的（`save_frames:=failed`），`all` 时成功的也有 |

`arrival_read.csv` 的列：`time,arrival,hold_lost` 加上和 `rc_read.csv` 相同的 tag 位置与读码结果列，可以用同一套脚本分析。
终端汇总读到、读不到都打 INFO，只有读码服务不在线时打 WARN。

**话题 `~result`**（`barcode_arrival_trigger/ArrivalReadResult`）：每次读码结束发布一条。

| 字段 | 含义 |
| --- | --- |
| `header` | 到位时刻（看到跳变的时刻） |
| `seq` | 到位序号，与日志 `arrival#N` 对应；被忽略、被门控放弃的跳变也占号 |
| `marker_id` | 到位时刻的 marker id（`marker_timeout_s` 内没有则 `-1`） |
| `camera_called` / `camera` | 是否按 `mode` 调用了相机读码；`BarcodeArray`（同相机节点 `~result`） |
| `serial_called` / `serial` | 是否按 `mode` 调用了读码器；`BarcodeResult`（`both` 时在相机读完之后才读） |
| `hold_lost_during_read` | 读码期间退出了 `POS_HOLD` |

没有总的 `success`，按两路各自的 `success` 判断；服务不在线时对应字段的 `error` 为 `unavailable (<原因>)`。

## 触发规则

- 只认「别的模式 → `POS_HOLD`」的跳变；节点启动时已停在 `POS_HOLD` 不会补读。
- 停在 `POS_HOLD` 期间只读一次，读失败也不重试；退出后再次进入才再读。
- 跳变那一刻要求 `/mavros/state` 为 `connected` + `armed` + `OFFBOARD` 且新鲜（`state_timeout_s` 内收到过），
  不满足就放弃本次到位（日志 `arrival#N 放弃：<原因>`），之后停在 `POS_HOLD` 也不补读。
- 上一次读码还没结束时的新跳变被忽略，日志 `arrival#N ignored`。
- 读码还没结束就退出了 `POS_HOLD`：不取消，读完照常发布，结果里标 `hold_lost_during_read`。

## 注意

- 读码节点和 AprilTag 的 `layout` 必须相同，tag 边长不一致时读码节点会警告，位置也不准。用 fusion 的 AprilTag 时
  tag 边长固定 43.7 mm，只能用 `qr_top_tag_bottom` / `dm_top_tag_bottom`。
- 想先让飞机自动移到码前、再手动拨 CH9 读码：终端 3 换成 `roslaunch barcode_rc_trigger manual_read.launch`，
  `rc_read.csv` 的 `control_mode` 列会记下每次拨杆时是否已在 `POS_HOLD`。两个 launch 不要同时起（读码节点重名）。
- 不要同时运行 DroneVision（与读码器共用 `/dev/ttyS1`）。
- 飞行中已经在跑 `ov13855_marker_tracking.launch` 时不能再起 `barcode_reader_camera/bringup.launch`（节点重名）；
  只要触发节点时用 `roslaunch barcode_arrival_trigger barcode_arrival_trigger.launch`，读码节点另起。

---

## 细节

### 参数文件

`config/barcode_arrival_trigger.yaml`，改完重启节点生效。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `control_mode_topic` | `/front_marker_control/control_mode` | 到位信号（`std_msgs/String`，latch，只在模式变化时发布） |
| `state_topic` | `/mavros/state` | 飞行门控状态 |
| `state_timeout_s` | 2.0 | 超过这么久收不到 state 视为不新鲜 |
| `transform_topic` | `/ov13855_apriltag_vision/debug/homogeneous_transform` | `marker_id` 和 tag 位置来源 |
| `marker_timeout_s` | 0.5 | 跳变时刻 marker 消息的最大年龄，超龄写 `-1` |
| `camera_service` | `/barcode_reader_camera/decode` | 相机读码服务 |
| `service_wait_s` | 0.5 | 等服务上线的最长时间，超时跳过该读码器 |
| `csv_dir` | `~/test_data/barcode` | 相机没给启动文件夹（只跑 serial）时 `arrival_read.csv` 的根目录；留空不写 CSV |
| `serial_service`（`serial.yaml`） | `/barcode_reader_serial/read` | 串口读码服务 |
| `serial_timeout_s`（`serial.yaml`） | 0.0 | 传给 `~read` 的超时，<=0 用读码器节点的 `read_timeout_ms` |

`mode` 是 launch 参数（默认 `camera`）；串口两项在 `config/serial.yaml`，只在 `mode` 为 `serial` / `both` 时加载。
`both` 先相机后读码器，两个都读（不是失败才用读码器）；先相机是为了不把读码器的补光/瞄准光拍进图里。
读码执行、日志和 CSV 格式复用 `barcode_rc_trigger` 的同一套代码（`runner` / `readers` / `record` / `tag_pose`）。

### 构建和测试

```bash
source /opt/ros/noetic/setup.bash
cd "$ORANGE_ROOT"
catkin build barcode_arrival_trigger
catkin run_tests barcode_arrival_trigger --no-deps && catkin_test_results build/barcode_arrival_trigger
```

测试不接触飞控、相机和读码硬件：`test_policy.py` 测跳变判定（启动基线、停留期间不重读、退出再进入、忙时忽略、`hold_lost` 标记）、
到位门控（state 未收到/超时/未连接/未解锁/非 OFFBOARD）和 marker 时效，
`node.test` 用假 `control_mode` / `state` / `transform` 话题和假读码服务跑完整节点
（触发、门控放弃、读码中退出、忙时忽略、服务离线、`marker_id` 新鲜与过期、`arrival_read.csv` 写进相机返回的启动文件夹）。
