# barcode_arrival_trigger

到位自动读码：`front_marker_control` 进入 `POS_HOLD`（到位定点）时自动读一次码，发布 `~result`，日志里打一块汇总。
本包只负责触发和发布；到位判定、失败重试、巡检航点调度都不在这里。手动拨 CH9 读码是 `barcode_rc_trigger`，两者互不干扰。

## 快速开始

只支持 OV13855 链路，两个终端：

```bash
source /opt/ros/noetic/setup.bash && source /home/orangepi/orange/devel/setup.bash
# 终端 1：相机 + AprilTag + 控制（到位判定）
roslaunch front_marker_control ov13855_marker_tracking.launch
# 终端 2：相机读码节点 + 串口读码器 + 到位触发节点
roslaunch barcode_arrival_trigger auto_read.launch
```

忘了参数怎么填：`roslaunch --ros-args barcode_arrival_trigger auto_read.launch`

## auto_read.launch 参数

| 参数 | 默认 | 可选值 |
| --- | --- | --- |
| `mode` | 空（用参数文件里的 `both`） | `camera` 只相机 / `serial` 只串口读码器 / `both` 先相机后读码器，两个都读 |
| `layout` | `qr_top_tag_bottom` | `qr_top_tag_bottom` / `dm_top_tag_bottom` / `drone_vision_default`，必须与终端 1 相同 |
| `save_frames` | `off` | `off` 不存图 / `failed` 只存失败 / `all` 都存 |
| `save_dir` | `~/barcode_frames` | 存图目录，`attempts.csv` 也在这里 |
| `config_file` | `config/barcode_arrival_trigger.yaml` | 到位触发节点参数文件，见「参数文件」 |

`auto_read.launch` 不管 `mode` 是什么都会启动串口读码器节点。版式说明见 `barcode_reader_camera` README「标签版式」。

## 结果在哪看

**终端**：每次到位一块汇总，格式同 `barcode_rc_trigger`，标题为 `arrival#N marker=M`，没有 tag 段。
读到、读不到都打 INFO，只有读码服务不在线时打 WARN。

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

**相机读码记录**：`~/barcode_frames/attempts.csv` 和存图目录，见 `barcode_reader_camera` README「读码记录」。

## 触发规则

- 只认「别的模式 → `POS_HOLD`」的跳变；节点启动时已停在 `POS_HOLD` 不会补读。
- 停在 `POS_HOLD` 期间只读一次，读失败也不重试；退出后再次进入才再读。
- 跳变那一刻要求 `/mavros/state` 为 `connected` + `armed` + `OFFBOARD` 且新鲜（`state_timeout_s` 内收到过），
  不满足就放弃本次到位（日志 `arrival#N 放弃：<原因>`），之后停在 `POS_HOLD` 也不补读。
- 上一次读码还没结束时的新跳变被忽略，日志 `arrival#N ignored`。
- 读码还没结束就退出了 `POS_HOLD`：不取消，读完照常发布，结果里标 `hold_lost_during_read`。

## 注意

- 两个终端的 `layout` 必须相同，tag 边长不一致时读码节点会警告，位置也不准。
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
| `transform_topic` | `/ov13855_apriltag_vision/debug/homogeneous_transform` | `marker_id` 来源 |
| `marker_timeout_s` | 0.5 | 跳变时刻 marker 消息的最大年龄，超龄写 `-1` |
| `mode` | `both` | `camera` / `serial` / `both`；launch 的 `mode:=` 不为空时覆盖它 |
| `camera_service` / `serial_service` | `/barcode_reader_camera/decode` / `/barcode_reader_serial/read` | 读码服务名 |
| `serial_timeout_s` | 0.0 | 传给 `~read` 的超时，<=0 用读码器节点的 `read_timeout_ms` |
| `service_wait_s` | 0.5 | 等服务上线的最长时间，超时跳过该读码器 |

`mode` 默认 `both`（不是失败才用读码器）：同一次到位拿两份结果对比；先相机是为了不把读码器的补光/瞄准光拍进图里。
读码执行和日志格式复用 `barcode_rc_trigger` 的同一套代码（`runner` / `readers`）。

### 构建和测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build barcode_arrival_trigger
catkin run_tests barcode_arrival_trigger --no-deps && catkin_test_results build/barcode_arrival_trigger
```

测试不接触飞控、相机和读码硬件：`test_policy.py` 测跳变判定（启动基线、停留期间不重读、退出再进入、忙时忽略、`hold_lost` 标记）、
到位门控（state 未收到/超时/未连接/未解锁/非 OFFBOARD）和 marker 时效，
`node.test` 用假 `control_mode` / `state` / `transform` 话题和假读码服务跑完整节点
（触发、门控放弃、读码中退出、忙时忽略、服务离线、`marker_id` 新鲜与过期）。
