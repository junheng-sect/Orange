# barcode_arrival_trigger

到位自动读码：`front_marker_control` 进入 `POS_HOLD`（到位定点）时，按 `mode` 调用一次
`barcode_reader_camera` 的 `~decode` 和/或 `barcode_reader_serial` 的 `~read`，发布结果
并在日志里汇总成一行。本节点只做触发与发布：到位判定、失败重试、兜底策略、巡检航点
调度都不在这里。

触发来源是控制节点新增的 `/front_marker_control/control_mode`（`std_msgs/String`，latch，
只在模式变化时发布）。CH9 手动触发仍是 `barcode_rc_trigger`，两者互不干扰：自动流程
不起 CH9 节点，地面/手持测试照常用它。

## 触发规则

- 只认「别的模式 → `POS_HOLD`」的跳变；节点启动后收到的第一条（通常是 latch 到的当前
  模式）只作基线，所以启动瞬间已经停在 `POS_HOLD` 不会补读。
- 停在 `POS_HOLD` 期间不重读，一次读码失败也不重试；退出后再次进入才再读一次。
- 跳变那一刻自订阅 `/mavros/state`，要求 `connected` + `armed` + `OFFBOARD` 且状态新鲜
  （超过 `state_timeout_s` 收不到即视为不新鲜）。不满足就放弃本次到位，只打日志，
  之后停在 `POS_HOLD` 也**不会**补读，必须退出再进入。
- 上一次读码还没结束时出现的新跳变按忙时忽略：占用到位序号但不读，
  日志提示 `arrival#N ignored`。
- 读码还没结束就退出了 `POS_HOLD`：不取消读码，读完后照常发布，结果里标
  `hold_lost_during_read`。
- `marker_id` 取跳变时刻 debug 话题 `/ov13855_apriltag_vision/debug/homogeneous_transform`
  上 `marker_timeout_s`（默认 0.5 s）以内最新的一条消息；过期或还没收到过就写 `-1`。

## 结果

`~result`（`barcode_arrival_trigger/ArrivalReadResult`）每次读码结束发布一条：

| 字段 | 含义 |
| --- | --- |
| `header` | 到位时刻（看到跳变的时刻） |
| `seq` | 到位序号，与日志 `arrival#N` 对应；被忙时忽略、被门控放弃的跳变也占号 |
| `marker_id` | 到位时刻的 marker id（须在 `marker_timeout_s` 内，否则 `-1`） |
| `camera_called` / `camera` | 本次是否调用了相机读码；`BarcodeArray`（`header` 为所解图像的 header，同相机节点 `~result`，没取到图像时为空） |
| `serial_called` / `serial` | 本次是否调用了读码器；`BarcodeResult`（`header` 为本地发起读码的时刻，`both` 时在相机读完之后） |
| `hold_lost_during_read` | 读码期间退出了 `POS_HOLD` |

到位序号只统计「别的模式 → `POS_HOLD`」的跳变本身，所以有放弃或忽略时结果里的 `seq`
会跳号，缺的号在日志里能查到原因（`arrival#N 放弃` / `arrival#N ignored`）。

没有总 `success` 字段：按两路各自的 `success` 判断；也不含 local 位置，到位目标看
`front_marker_control` 自己的状态话题。`camera_called` / `serial_called` 只表示按 `mode`
是否调用了这一路，与服务是否在线、是否解码成功无关；服务不在线时对应字段的 `error`
记为 `unavailable (<原因>)`，其余字段保持默认值。

日志汇总示例（全部成功打 INFO，否则打 WARN）：

```text
arrival#3 marker=0 camera: ok report="QR=1/1" [QRCode]"A20260609001" 480ms | serial: ok "NEXT9017324" 720ms
```

## 启动

只支持 OV13855 链路。相机、视觉、控制在一个终端，读码在另一个终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash

# 终端 1：相机 + AprilTag + 控制（到位判定）
roslaunch front_marker_control ov13855_marker_tracking.launch

# 终端 2：相机读码节点 + 串口读码器 + 到位触发节点
roslaunch barcode_arrival_trigger auto_read.launch
roslaunch barcode_arrival_trigger auto_read.launch mode:=serial     # 只要读码器
```

两个终端默认都用 `layout:=qr_top_tag_bottom` 标签版式；换标签时两边传同一个 `layout:=<名字>`（见
`barcode_reader_camera` README 的「标签版式」），不一致时读码节点会警告 tag 边长不符。

只起触发节点（读码节点另起，例如飞行中已经在跑 `ov13855_marker_tracking.launch` 时
不能重复起 `barcode_reader_camera/bringup.launch`）：

```bash
roslaunch barcode_arrival_trigger barcode_arrival_trigger.launch
```

不要同时运行 DroneVision（与读码器共用 `/dev/ttyS1`）。

## 参数

`config/barcode_arrival_trigger.yaml`，修改后重启节点生效；也可用
`config_file:=/absolute/path.yaml` 覆盖。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `control_mode_topic` | `/front_marker_control/control_mode` | 到位信号（`std_msgs/String`，latch） |
| `state_topic` | `/mavros/state` | 飞行门控状态 |
| `state_timeout_s` | 2.0 | 超过这么久收不到 state 视为不新鲜 |
| `transform_topic` | `/ov13855_apriltag_vision/debug/homogeneous_transform` | `marker_id` 来源 |
| `marker_timeout_s` | 0.5 | 跳变时刻 marker 消息的最大年龄，超龄写 `-1` |
| `mode` | `both` | `camera` / `serial` / `both`（先相机后读码器，两个都读） |
| `camera_service` / `serial_service` | `/barcode_reader_camera/decode` / `/barcode_reader_serial/read` | 读码服务名 |
| `serial_timeout_s` | 0.0 | 传给 `~read` 的超时，<=0 用读码器节点的 `read_timeout_ms` |
| `service_wait_s` | 0.5 | 等服务上线的最长时间，超时跳过该读码器 |

`mode` 默认 `both`（不是失败才用读码器）：同一次到位拿两份结果对比；先相机的原因是
相机节点收到请求后等下一帧新图，读码器的补光/瞄准光不会拍进这一帧。

## 构建和测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build barcode_arrival_trigger
catkin run_tests barcode_arrival_trigger --no-deps
catkin_test_results build/barcode_arrival_trigger
```

测试不接触飞控、相机和读码硬件：`test_policy.py` 测跳变判定（启动基线、停留期间不重读、
退出再进入、忙时忽略、`hold_lost` 标记）、到位门控（state 未收到/超时/未连接/未解锁/
非 OFFBOARD）和 marker 时效（`marker_timeout_s` 内才用，过期或没收到写 `-1`），
`node.test` 用假 `control_mode`/`state`/`transform` 话题和假读码服务跑完整节点
（触发、门控放弃、读码中退出、忙时忽略、服务离线、`marker_id` 新鲜与过期）。

## 相关包

- 到位信号：`front_marker_control` 的 `/front_marker_control/control_mode`（latch，
  模式变化时发布），配置项 `control_mode_topic`。
- 读码执行与日志格式复用 `barcode_rc_trigger.runner`（CH9 触发节点的同一套流程）。
