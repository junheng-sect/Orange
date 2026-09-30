# barcode_rc_trigger

遥控器开关（默认 CH9）触发一次读码。本节点只作为服务客户端：拨一下开关，按 `mode` 调用
`barcode_reader_camera` 的 `~decode` 和/或 `barcode_reader_serial` 的 `~read`，并在日志里汇总成一行。
两个读码节点和 MAVROS 需要另外启动，本包不改动它们。

触发判据沿用 9.1 CH9 弹簧开关拍照：`/mavros/rc/in` 的 CH9（`channels[8]`）低于 1600 布防，
高于 1700 并持续 0.05 s 触发一次；按住不放不会重复触发，回到低位后才能再次触发；
节点启动或 RC 重连时开关已在高位不会触发。

## 启动

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
# 前提：MAVROS 在跑（/mavros/rc/in 有数据），读码节点已启动
roslaunch barcode_rc_trigger barcode_rc_trigger.launch            # 用 yaml 里的 mode（默认 both）
roslaunch barcode_rc_trigger barcode_rc_trigger.launch mode:=serial
```

读码节点的启动方式：
- 地面单独测相机：`roslaunch barcode_reader_camera bringup.launch`（相机 + AprilTag + 读码节点）。
- 飞行中（已经在跑 `ov13855_marker_tracking.launch`）：只起 `roslaunch barcode_reader_camera barcode_reader_camera.launch`，
  不能再起 bringup.launch，否则相机和 AprilTag 节点重名会冲突。
- 串口读码器：`roslaunch barcode_reader_serial barcode_reader.launch`。

先确认开关的实际 PWM：`rostopic echo -n1 /mavros/rc/in`，看 `channels[8]` 的低位和高位是否分别在 1600 和 1700 两侧。

## 行为

- `mode`：`camera` 只调相机；`serial` 只调读码器；`both` 先相机后读码器，两个都调（不是失败才用读码器），
  同一次拨杆拿两份结果对比。先相机的原因：相机节点收到请求后等下一帧新图，读码器的补光或瞄准光不会拍进这一帧。
- 读码在工作线程里执行；上一次还没结束时再拨，这一下会被忽略，日志提示 `pulse#N ignored`。
- 某个服务 `service_wait_s` 内不在线时跳过它，另一个照常执行。
- 每次开始读码时发布 `~pulse`（`std_msgs/UInt32`，拨杆序号，被忽略的拨杆也占序号但不发布），方便在 bag 里对齐；
  详细结果看两个读码节点各自的 `~result`。
- 日志汇总示例：
  `pulse#3 camera: ok report="QR=1/1" [QRCode]"A20260609001" 480ms | serial: ok "NEXT9017324" 720ms`
  （全部成功打 INFO，否则打 WARN）。
- CH9 数值不在 800~2200（通道缺失或未使用）或超过 `rc_timeout_s` 收不到时视为 RC 无效，撤防，必须重新见到低位。

## 参数

`config/barcode_rc_trigger.yaml`，修改后重启节点生效；也可用 `config_file:=/absolute/path.yaml` 覆盖。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `rc_topic` | `/mavros/rc/in` | RC 话题（`mavros_msgs/RCIn`） |
| `rc_channel` | 9 | 1-based 通道号 |
| `arm_below` / `trigger_above` | 1600 / 1700 | 布防 / 触发阈值（us） |
| `hold_s` | 0.05 | 高位持续多久才算一次拨杆 |
| `rc_timeout_s` | 1.0 | 超过这么久没有有效 CH9 视为 RC 丢失 |
| `mode` | `both` | `camera` / `serial` / `both` |
| `camera_service` / `serial_service` | `/barcode_reader_camera/decode` / `/barcode_reader_serial/read` | 读码服务名 |
| `serial_timeout_s` | 0.0 | 传给 `~read` 的超时，<=0 用读码器节点的 `read_timeout_ms` |
| `service_wait_s` | 0.5 | 等服务上线的最长时间 |

## 构建和测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build barcode_rc_trigger
catkin run_tests barcode_rc_trigger --no-deps
catkin_test_results build/barcode_rc_trigger
```

测试不接触飞控和读码硬件：`test_policy.py` 测触发状态机（上电高位、去抖、按住、重新布防、中间区、RC 丢失），
`test_runner.py` 测调用顺序、单个服务失败不影响另一个和日志格式，`node.test` 用假 RC 话题和假读码服务跑完整节点
（触发、忙时忽略、RC 丢失撤防、服务离线）。
