# barcode_rc_trigger

遥控器开关（默认 CH9）触发一次读码。本节点只作为服务客户端：拨一下开关，按 `mode` 调用
`barcode_reader_camera` 的 `~decode` 和/或 `barcode_reader_serial` 的 `~read`，并在日志里输出一块 YAML 风格的汇总。
两个读码节点和 MAVROS 需要另外启动，本包不改动它们。
每次拨杆同时记下 tag 位置，写进汇总日志和 CSV，用来测在什么位置相机/读码器能读成功。

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

手动测读码范围用 `manual_read.launch`（相机读码节点 + 本节点，`mode:=both`/`serial` 时加串口读码器）。默认 `camera:=false`，配合
`/home/orangepi/mid360s_ws/scripts/fusion.sh`：fusion 默认已起 MAVROS、OV13855 相机和 AprilTag，本 launch 不再起相机，
否则节点重名会互相踢掉。

```bash
# 终端 1：fusion 照常启动
cd /home/orangepi/mid360s_ws/scripts && ./fusion.sh
# 终端 2：fusion 起来后
roslaunch barcode_rc_trigger manual_read.launch                   # 默认 layout:=qr_top_tag_bottom、mode:=camera
roslaunch barcode_rc_trigger manual_read.launch mode:=both         # 相机 + 串口读码器都调
```

- 不跑 fusion 单独测试（地面测试）：`manual_read.launch camera:=true`，由本 launch 起相机 + AprilTag；
  MAVROS 不并进来（它平时也一直打印，会把终端弄乱），单独开一个终端起。板端系统的 `mavros/launch/px4.launch`
  默认值已改成与 fusion.launch 一致（`fcu_url` `/dev/ttyS3:921600`、`fcu_protocol` v1.0，原文件备份为 `px4.launch.orig`），
  直接起即可；apt 升级 mavros 会覆盖这个改动，届时显式传 `fcu_url:=/dev/ttyS3:921600 fcu_protocol:=v1.0`。

  ```bash
  # 终端 1：MAVROS
  roslaunch mavros px4.launch
  # 终端 2：相机 + AprilTag + 读码节点 + 本节点；默认只测相机，测串口读码器加 mode:=serial 或 mode:=both
  export ROS_IP=192.168.31.32   # 本机要远程看话题时需要
  roslaunch barcode_rc_trigger manual_read.launch camera:=true
  roslaunch barcode_rc_trigger manual_read.launch camera:=true mode:=serial
  ```

  此时不能再跑 `ov13855_marker_tracking.launch`、`bringup.launch` 或带相机的 fusion。
- `mode`：本 launch 默认 `camera`（串口读码器效果差，日常只用相机），此时不启动串口读码器节点；
  `mode:=both` / `mode:=serial` 才启动它。单独起 `barcode_rc_trigger.launch` 时仍用 yaml 里的 `mode`（`both`）。
- `save_frames:=failed|all`：相机读码节点按次保存原图、位姿和策略日志到 `save_dir`（默认 `~/barcode_frames`），
  每次读码另在 `attempts.csv` 记一行；见 `barcode_reader_camera` README「读码记录」。
- `quiet:=true`（默认）：本 launch 起的相机、AprilTag、读码节点用 `output=log`，INFO 只进 `~/.ros/log`，
  WARN/ERROR 仍打到终端，终端里基本只剩本节点的拨杆汇总；`quiet:=false` 恢复全部打印。
- `camera:=false` 时 AprilTag 用的是 fusion 那边的默认视觉 YAML（tag 43.7 mm），只和 tag 43.7 mm 的版式一致：
  `layout:=qr_top_tag_bottom`（QR 标签，默认）或 `layout:=dm_top_tag_bottom`（DataMatrix 标签）；
  换其他 tag 边长的 layout 要改用 `camera:=true`，并让 fusion 带 `--no-camera` 启动。

读码节点的启动方式：
- 地面单独测相机：`roslaunch barcode_reader_camera bringup.launch`（相机 + AprilTag + 读码节点）。
- 飞行中（已经在跑 `ov13855_marker_tracking.launch`）：只起 `roslaunch barcode_reader_camera barcode_reader_camera.launch`，
  不能再起 bringup.launch，否则相机和 AprilTag 节点重名会冲突。
- 串口读码器：`roslaunch barcode_reader_serial barcode_reader.launch`。

先确认开关的实际 PWM：`rostopic echo -n1 /mavros/rc/in`，看 `channels[8]` 的低位和高位是否分别在 1600 和 1700 两侧。

## 行为

- `mode`：`camera` 只调相机；`serial` 只调读码器；`both` 先相机后读码器，两个都调（不是失败才用读码器），
  同一次拨杆拿两份结果对比。先相机的原因：相机节点收到请求后等下一帧新图，读码器的补光或瞄准光不会拍进这一帧。
- 读码在工作线程里执行；上一次还没结束时再拨，这一下会被忽略，日志提示 `pulse#N ignored`。相机第一帧没读出时会换新帧
  重试，读不出要约 2 s 才返回（`barcode_reader_camera` 的 `retry.timeout_ms`）。
- 某个服务 `service_wait_s` 内不在线时跳过它，另一个照常执行。
- 每次开始读码时发布 `~pulse`（`std_msgs/UInt32`，拨杆序号，被忽略的拨杆也占序号但不发布），方便在 bag 里对齐；
  详细结果看两个读码节点各自的 `~result`。
- 每次拨杆一块汇总（风格同 `rostopic echo`，以 `---` 结尾；只跑一路时只有那一段）：

  ```
  [INFO] [1791699346.533133]: pulse#1
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
  serial:
    ok: false
    error: no_read
    ms: 3019
  ---
  ```

  读到、读不到都是 INFO（读不到是测距实验的正常结果）；只有读码服务不在线时打 WARN，该段为
  `ok: false`、`error: unavailable`、`detail: "..."`。`barcode_arrival_trigger` 共用同一格式（标题 `arrival#N marker=M`，没有 tag 段）。
- tag 位置来自 `transform_topic`（AprilTag 节点的 `debug/homogeneous_transform`），是 tag 中心在相机光学系下的坐标
  （x 右、y 下、z 前，m），`dist` 为直线距离，`tilt_deg` 为 tag 法向与相机光轴夹角（正对为 0）。
  `source: img` 是相机读码所用那一帧图像的位姿（换帧重试读出时是读出的那一帧）；相机没取到图（或只跑 serial）时用拨杆时刻最新一帧，记为 `source: pulse`；
  拨杆时刻 `tag_timeout_s` 内没有 tag 则为 `tag: none`。both 模式下读码器在相机之后才读，期间若在移动，位置以相机那一帧为准。
- 每次拨杆往 `csv_dir/rc_read_<启动时间>.csv` 追加一行（每次启动一个新文件），列：
  `time,pulse,tag_src,marker_id,x,y,z,dist,tilt_deg,camera_ok,camera_error,camera_report,camera_codes,camera_ms,serial_ok,serial_error,serial_code,serial_ms`；
  未调用的那一路留空，服务不在线时 `*_ok=0`、`*_error=unavailable`。
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
| `transform_topic` | `/ov13855_apriltag_vision/debug/homogeneous_transform` | tag 位姿来源，留空不记录位置 |
| `tag_timeout_s` | 0.5 | 拨杆时刻最新 tag 的最大年龄 |
| `csv_dir` | `~/barcode_rc_logs` | CSV 目录，留空不写 |

## 构建和测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build barcode_rc_trigger
catkin run_tests barcode_rc_trigger --no-deps
catkin_test_results build/barcode_rc_trigger
```

测试不接触飞控和读码硬件：`test_policy.py` 测触发状态机（上电高位、去抖、按住、重新布防、中间区、RC 丢失），
`test_tag_record.py` 测 tag 位置换算、按图像时间戳匹配和 CSV 行，
`test_runner.py` 测调用顺序、单个服务失败不影响另一个、汇总块格式和日志级别，`node.test` 用假 RC 话题和假读码服务跑完整节点
（触发、忙时忽略、RC 丢失撤防、服务离线）。
