# barcode_reader_serial

在当前 Orange 检出中执行 `export ORANGE_ROOT="$(git rev-parse --show-toplevel)"`，下文命令以此定位仓库。

串口读码器（`/dev/ttyS1`）：每次调用 `~read` 触发读码器一次，等一帧回复，返回读到的码。
平时由 `barcode_rc_trigger`（`mode:=serial` / `both`）或 `barcode_arrival_trigger` 调用。
移植自 Drone-Vision C# 程序的 `BarcodeReaderSerial`（`camera_then_serial` 的串口那一路）。

## 快速开始

```bash
source /opt/ros/noetic/setup.bash && source "$ORANGE_ROOT"/devel/setup.bash
roslaunch barcode_reader_serial barcode_reader.launch
# 另一个终端：触发一次，看结果
rosservice call /barcode_reader_serial/read "timeout_s: 0.0"
rostopic echo /barcode_reader_serial/result
```

读码器和 DroneVision 共用 `/dev/ttyS1`，不能同时运行；启动前确认 `pgrep -af DroneVision` 没有输出。

忘了参数怎么填：`roslaunch --ros-args barcode_reader_serial barcode_reader.launch`

## launch 参数

| 参数 | 默认 | 可选值 |
| --- | --- | --- |
| `config_file` | `config/barcode_reader.yaml` | 读码器参数文件，见「参数文件」 |
| `output` | `screen` | `screen` 日志打到终端 / `log` INFO 只进 `~/.ros/log`，WARN/ERROR 仍打到终端 |

## 读码结果

`~read` 返回 `success / code / error / elapsed_s / request_seq / raw_hex`，每次结束也在 `~result` 发布一条（包括失败）。

| `error` | 含义 |
| --- | --- |
| 空 | 读到了，`code` 是码内容 |
| `no_read` | 读码器回复读不到码（默认 `NR`，约 3 s 才回），原始回复见 `raw_hex` |
| `timeout` | 超时没有回复 |
| `serial_unavailable` | 串口打不开 |
| `busy` | 上一次还没结束 |
| `cancelled` | 被 `~cancel` 打断或节点退出 |
| `empty_frame` | 读码器回了空帧 |
| `write_failed` / `io_error` | 串口读写出错（会关闭串口，下次 `~read` 时重新打开） |
| `not_configured` / `invalid_timeout` | 触发指令为空 / 请求的超时非法 |

连续 `no_read` 时先看读码器的瞄准光、补光有没有落在码上：2026-10-08 现场读码器装反，光斑在 QR 下方约 84 mm，62 次全部 `NR`；
装正后 105 次全部读到，触发到回码 0.2~0.5 s。

## 参数文件

`config/barcode_reader.yaml`，改完重启节点生效。参数名与 Drone-Vision `config.json` 的 `barcode_decoder.serial_reader` 一致，
默认值取自板端 `publish/config.json`。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `port` | `/dev/ttyS1` | 40pin UART1_M1（RX=27 / TX=28），需启用 `uart1-m1` 覆盖；`orangepi` 已在 dialout 组 |
| `baud` / `data_bits` / `parity` / `stop_bits` | 115200 / 8 / None / One | parity 可选 None/Even/Odd/Mark/Space，stop_bits 可选 One/OnePointFive/Two |
| `trigger_hex` | `02 F4 03` | 触发指令；置空时改用文本指令 `trigger_command` |
| `release_hex` | `02 F5 03` | 释放指令；置空则不下发 |
| `reply_start_hex` / `reply_end_hex` | 空 / 空 | 都置空为纯文本模式，按 CR/LF 判帧、空行跳过；都设置时取起止符之间的内容，起始符前的字节视为噪声丢弃 |
| `no_read_reply` | `NR` | 读不到码时的回复；帧去掉首尾空白后与之完全相同即返回 `no_read`；置空则不识别，按普通条码返回 |
| `read_timeout_ms` | 3500 | 单次读码超时，范围 (0, 30000]。实测读到时 0.5~2.7 s 回码，读不到时约 3.02~3.05 s 回 `NR`；要大于这个时间，否则慢的成功会判 `timeout`、读不到时也拿不到 `no_read` |

---

## 细节

### 接口

- `~read`（`barcode_reader_serial/ReadBarcode`）：`timeout_s <= 0` 时用 `read_timeout_ms`，上限 30 s；阻塞直到收到一帧、超时或被取消。
- `~cancel`（`std_srvs/Trigger`）：打断正在进行的 `~read`，被打断的请求返回 `cancelled`；没有进行中的请求时 `success=false`。
- `~result`（`barcode_reader_serial/BarcodeResult`）：每次 `~read` 结束后发布一条（包括失败）。

### 行为（与 C# 版一致）

- 同一时间只允许一个请求；触发前清空串口接收缓冲，并丢弃上次请求的残留字节。
- 不在请求期间收到的数据（超时后才到的回复、设备主动上报的数据）一律丢弃，不会被下一次请求读到。
- 成功、超时、空帧、读不到码（`no_read`）、取消以及节点退出时都会下发 `release_hex`。
- 日志级别：成功和 `no_read`（读码器正常回复读不到）打 INFO；超时、空帧、串口错误等打 WARN。
- 启动时串口打不开只告警，节点继续运行；之后每次 `~read` 时重试打开。
- `code` 为去掉帧头帧尾和首尾空白后按 UTF-8 解码的文本（非法字节替换为 U+FFFD），原始字节见 `raw_hex`。
- 本包只负责读码器，不包含相机解码、到位判定状态机和 UDP 飞控协议。

### 构建和测试

```bash
source /opt/ros/noetic/setup.bash
cd "$ORANGE_ROOT"
catkin build barcode_reader_serial
catkin run_tests barcode_reader_serial --no-deps && catkin_test_results build/barcode_reader_serial
```

测试不接触实物串口：`test_protocol.py` 测分帧，`test_reader.py` 用内存假串口测读码逻辑，
`test_pty_device.py` 用伪终端模拟读码器、经真实 pyserial 端到端验证，`node.test` 用伪终端跑完整的 ROS 节点
（读码成功、读不到码、超时、忙、取消、`~result` 发布，以及触发和释放次数）。
