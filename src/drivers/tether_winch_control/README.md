# 两档系留绕线器控制

在当前 Orange 检出中执行 `export ORANGE_ROOT="$(git rev-parse --show-toplevel)"`，下文命令以此定位仓库。

根据 MID360S 定位判断无人机与固定出线点的距离变化，通过 TCP 向绕线器发送
`DUTY=<整数>#`。电机始终向收线方向施力，DUTY 越大力矩越大；放线由无人机拉动。
本包不控制飞机，不发送飞行设定值。

## 参数

日常修改 `config/winch.yaml`，保存后重启绕线器节点生效；没有运行时热更新。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `auto_enable` | `true` | 启动后定位与通信就绪时自动启用；设为 false 恢复手动启用 |
| `transport` | `tcp` | 实机 TCP；`mock` 不产生网络流量 |
| `controller.low_duty` | 100 | 放线、静止、等距离横移时的低档 |
| `controller.high_duty` | 300 | 持续明确靠近出线点时的收线档 |
| `controller.fallback_duty` | 100 | 启动禁用、故障、停用和正常退出时的输出 |
| `tcp.host` / `tcp.port` | `192.168.31.87` / 8080 | 设备 IPv4 地址、端口 |
| `tcp.timeout_sec` | 0.2 秒 | 分别限制连接和发送；允许大于 0、不超过 5 秒 |
| `tcp.resend_sec` | 1 秒 | 稳定档位的重发间隔，最小 0.05 秒 |
| `odom_topic` | `/fastlio/odom_world` | `nav_msgs/Odometry` |
| `controller.frame_id` / `child_frame_id` | `drone_odom` / `drone_base` | 严格检查消息坐标系 |
| `controller.anchor` | `[0, 0, 0]` | 固定出线点在当次 drone_odom 坐标系中的坐标（米） |
| `controller.attachment_offset` | `[0, 0, 0]` | 雷达 IMU 到挂线点的偏移，按机体轴表达（米） |

三个 DUTY 均必须是 **0～500 的整数**，拒绝浮点数、布尔值、越界值，且要求
`low_duty < high_duty`。这是设备原始指令值，不是 0～1 比例，也不是测量力矩。
允许配置 0 不代表本包已验证设备 DUTY=0 时的机械行为。

MID360S 的重力对齐桥接输出姿态对应机体，但位置基点仍是雷达 IMU。
`anchor` 必须按本次定位原点填写，不能默认 FAST-LIO 原点就在绕线器出线点。
FAST-LIO 重启或重定位后，需要核对坐标再启用；零偏移仅适用于实际几何关系符合时。
挂线点位置计算为 `p_odom + R(orientation) * attachment_offset`。

## 判定与异常行为

使用源时间戳窗口内的距离最小二乘拟合计算径向速度，不读取 `Odometry.twist`。
现有判定参数保留：窗口 0.2 秒、至少 3 个样本；速度小于 -0.05 m/s 连续 0.15 秒
进入 HIGH，回到 -0.02 m/s 及以上退出 HIGH；距离出线点不超过 0.1 米禁止 HIGH。
定位超时默认 0.5 秒；时间戳异常、帧错误、位置跳变、无效四元数和样本不足均锁定故障。
跳变上限为 `position_jump_slack_m + max_position_speed_mps * dt`，默认 0.05 米和 3 m/s。
源数据和接收年龄都被检查；看门狗使用墙钟线程，ROS `/clock` 停止也会检测接收超时。
拟合窗口及迟滞会产生切换延迟，距离是几何间距，不是实际放线长度或张力。

| 状态 | 自动目标 | TCP 行为 |
| --- | --- | --- |
| DISABLED | 无 | 发送 fallback_duty |
| LOW | low_duty | 发送低档 |
| HIGH | high_duty | 发送高档 |
| FAULT | 无，保持锁定 | 尝试发送 fallback_duty |

默认 `auto_enable: true`：**TCP 模式启动即发送 fallback_duty**，等待足够新鲜定位及
TCP 写入成功后，自动进入 LOW，随后按距离变化切换，无需调用启用服务。
首次启用前若定位或网络暂未就绪，会持续等待恢复；首次自动启用后不再自动重试启用。
运行期间故障仍保持锁定，可排查后重启节点或调用启用服务恢复。
显式调用停用服务会取消尚未完成的启动自动启用，也不会立即被自动打开。
通信失败锁定故障，即使里程计继续到达，也只重试低档，直到通信恢复并人工重新启用。
正常退出在发送线程结束前进行一次有超时限制的 fallback 发送；强制杀进程、断电或
网络中断时无法保证低档送达，设备可能保持上一条指令。设备端看门狗不在本次实现范围内。

## TCP 协议和状态

等价发送示例（会驱动真实设备，只在准备好时执行）：

```bash
printf '%s' 'DUTY=270#' | nc -N 192.168.31.87 8080
```

使用 Python 标准库 socket，无需运行 shell 或安装 nc。每条命令新建连接，发送 ASCII
文本，不带换行，随后关闭写端并关闭连接。不等待或解释设备应答。
独立线程只保留最新目标，变化时立即唤醒，稳定时周期重发；连接期间目标被替换则放弃
旧值，不积压历史指令。已开始发送的单条指令无法撤回，后续发送使用最新值。
退出低档与在途发送串行执行，不会在最终低档之后补发高档。

- 启停服务：`/tether_winch_control/set_enabled`，`std_srvs/SetBool`。
- 状态话题：`/tether_winch_control/status`，`WinchStatus`，默认 20 Hz。
- `target_duty` 表示自动控制目标；DISABLED/FAULT 时 `command_valid=false`、目标为 NaN，
  此时实际发送策略由 `fallback_duty` 决定。
- `last_sent_duty`、`last_send_time` 是最近成功 socket 写入值及 Unix 墙钟时间，初始为 NaN/0。
  `communication_error` 是最新通信错误，成功写入后清空；清空不解除 FAULT。
- 没有设备执行回报：`hardware_feedback_valid=false`，`applied_duty=NaN`。
  服务返回成功只代表接受启停请求，不代表电机已执行。mock 的发送状态始终 NaN/0。

## 板端运行

先确认 MID360S 正常发布定位。本 launch 只启动绕线器，不启动或重启雷达。
首次启动前修改 anchor 和 attachment_offset。以下实机命令启动即发送低档：

```bash
ssh orange
source /opt/ros/noetic/setup.bash
cd "$ORANGE_ROOT"
catkin build tether_winch_control
source "$ORANGE_ROOT"/devel/setup.bash
source "$ORANGE_ROOT"/scripts/ros_net.sh
roslaunch tether_winch_control tether_winch.launch
```

另一个板端终端：

```bash
source /opt/ros/noetic/setup.bash
source "$ORANGE_ROOT"/devel/setup.bash
source "$ORANGE_ROOT"/scripts/ros_net.sh
rostopic echo /tether_winch_control/status
# 默认自动启用，无需以下启用命令；它保留用于故障恢复或 auto_enable=false：
rosservice call /tether_winch_control/set_enabled 'data: true'
rosservice call /tether_winch_control/set_enabled 'data: false'
```

观察定位判断而不发送网络命令：

```bash
roslaunch tether_winch_control tether_winch.launch simulation_values:=true
```

`simulation_values:=true` 明确覆盖 transport 为 mock；仍需外部定位。

## 自动验证（不连接真实绕线器）

板端执行：

```bash
source /opt/ros/noetic/setup.bash
source "$ORANGE_ROOT"/devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s "$ORANGE_ROOT"/src/drivers/tether_winch_control/test -v
cd "$ORANGE_ROOT"
catkin run_tests tether_winch_control
catkin_test_results build/tether_winch_control/test_results
```

单元测试使用 localhost 随机端口。rostest 使用隔离的 ROS Master 和 localhost:18080，
注入 drone_odom/drone_base 里程计，验证实际 YAML、状态、服务、掉定位、错误帧、TCP 中断恢复、
人工重新启用和正常退出。测试不使用硬件 IP；请保持本机测试端口 18080 空闲。

无需 ROS 的轨迹模拟：

```bash
PYTHONPATH="$ORANGE_ROOT"/src/drivers/tether_winch_control/src \
python3 "$ORANGE_ROOT"/src/drivers/tether_winch_control/scripts/simulate_winch \
  --low-duty 100 --high-duty 300 --csv /tmp/tether_winch_trace.csv
```

模拟只使用 mock，命令行参数独立于 YAML。构建和自动测试不能替代设备通信、实际力矩、
温升及飞行测试。本包没有张力、电流或放线长度反馈。
