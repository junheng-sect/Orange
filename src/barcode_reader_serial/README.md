# barcode_reader_serial

串口读码器 ROS Noetic 包，移植自 Drone-Vision C# 程序中的 `BarcodeReaderSerial`
（`camera_then_serial` 的串口兜底那一路）。每次调用 `~read` 触发读码器一次，等待一帧回复后下发释放指令。
本包只负责读码器，不包含相机解码、到位判定状态机和 UDP 飞控协议。

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
roslaunch barcode_reader_serial barcode_reader.launch
# 另一个终端
rosservice call /barcode_reader_serial/read "timeout_s: 0.0"
rostopic echo /barcode_reader_serial/result
```

读码器与 DroneVision 共用 `/dev/ttyS1`，两者不能同时运行；启动前确认 `pgrep -af DroneVision` 为空。

## 接口

- `~read`（`barcode_reader_serial/ReadBarcode`）：`timeout_s <= 0` 时用配置的 `read_timeout_ms`，上限 30 s。
  阻塞直到收到一帧、超时或被取消，返回 `success / code / error / elapsed_s / request_seq / raw_hex`。
- `~cancel`（`std_srvs/Trigger`）：打断正在进行的 `~read`，被打断的请求返回 `cancelled`；没有进行中的请求时 `success=false`。
- `~result`（`barcode_reader_serial/BarcodeResult`）：每次 `~read` 结束后都发布一条（包括失败）。

`error` 取值：空字符串表示成功；`serial_unavailable` 串口打不开；`busy` 已有请求在进行；
`not_configured` 触发指令为空；`timeout` 超时无回复；`empty_frame` 读码器回了空帧；
`no_read` 读码器回复“读不到码”（即 `no_read_reply`，默认 `NR`），原始回复仍见 `raw_hex`；`cancelled` 被取消或节点退出；
`write_failed` / `io_error` 串口读写出错（会关闭串口，下次 `~read` 时重新打开）；`invalid_timeout` 请求的超时非法。

连续 `no_read`（每次约 3 s 回 `NR`）时先看读码器的瞄准光/补光是否落在码上：2026-10-08 现场读码器装反，光斑在 QR 下方约
84 mm，62 次全部 `NR`；装正后 105 次全部读到，触发到回码 0.2~0.5 s。

## 行为（与 C# 版一致）

- 同一时间只允许一个请求；触发前清空串口接收缓冲，并丢弃上次请求的残留字节。
- 不在请求期间收到的数据（超时后才到的回复、设备主动上报的数据）一律丢弃，不会被下一次请求读到。
- 成功、超时、空帧、读不到码（`no_read`）、取消以及节点退出时都会下发 `release_hex`。
- 启动时串口打不开只告警，节点继续运行；之后每次 `~read` 时重试打开。
- `code` 为去掉帧头帧尾和首尾空白后按 UTF-8 解码的文本（非法字节替换为 U+FFFD），原始字节见 `raw_hex`。

## 参数

日常修改 `config/barcode_reader.yaml`，重启节点生效；也可用 `config_file:=/absolute/path.yaml` 覆盖。
参数名与 Drone-Vision `config.json` 的 `barcode_decoder.serial_reader` 保持一致，默认值取自板端 `publish/config.json`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `/dev/ttyS1` | 40pin UART1_M1（RX=27 / TX=28），需启用 `uart1-m1` 覆盖；`orangepi` 已在 dialout 组 |
| `baud` / `data_bits` / `parity` / `stop_bits` | 115200 / 8 / None / One | parity 可选 None/Even/Odd/Mark/Space，stop_bits 可选 One/OnePointFive/Two |
| `trigger_hex` | `02 F4 03` | 触发指令；置空时改用文本指令 `trigger_command` |
| `release_hex` | `02 F5 03` | 释放指令；置空则不下发 |
| `reply_start_hex` / `reply_end_hex` | 空 / 空 | 两者都置空为纯文本模式，按 CR/LF 判帧，空行跳过；两者都设置时取起止符之间的内容，起始符前的字节视为噪声丢弃 |
| `no_read_reply` | `NR` | 读码器读不到码时的回复；帧内容去掉首尾空白后与之完全相同即返回 `no_read`（不算成功）；置空则不识别，按普通条码返回 |
| `read_timeout_ms` | 3500 | 单次读码超时，范围 (0, 30000]。实测读到时 0.5~2.7 s 回码，读不到时读码器约 3.02~3.05 s 回 `NR`；超时要大于这个时间，否则慢的成功会判 `timeout`、读不到时也只能拿到 `timeout` 而不是 `no_read` |

## 构建和测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build barcode_reader_serial
catkin run_tests barcode_reader_serial --no-deps
catkin_test_results build/barcode_reader_serial
```

测试不接触实物串口：`test_protocol.py` 测分帧，`test_reader.py` 用内存假串口测读码逻辑，
`test_pty_device.py` 用伪终端模拟读码器、经真实 pyserial 端到端验证，`node.test` 用伪终端跑完整的 ROS 节点
（读码成功、读不到码、超时、忙、取消、`~result` 发布，以及触发和释放次数）。
