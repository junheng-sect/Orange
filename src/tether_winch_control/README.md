# 两档系留绕线器控制（离线版）

本包根据定位自动选择放线低档 / 收线高档 PWM。电机始终向收线方向施力，
放线由飞机或手持人员拉动卷筒完成。目标是先通过手持地面试验确定两档值和适用范围，
后续再考虑连续调节。本包不控制飞机、不切换飞行模式、不发送飞行设定值。

**目前仅实现 mock 通信端，不连接 ESP32，不输出实际 PWM，也没有实际电机回报。**
曾使用的地址 `192.168.31.87` 只作为接入线索；协议、端口、指令、占空比分辨率、
回报和失联行为均未确认。任何非 `mock` 通信模式都会拒绝启动。

## 功能和限制

| 状态 | 行为 |
| --- | --- |
| `DISABLED` | 默认状态，没有有效控制命令，仍积累定位历史 |
| `LOW` | 远离、静止、等距离横移，以及靠近尚未满足进入条件时输出低档 |
| `HIGH` | 持续明确靠近时输出高档 |
| `FAULT` | 定位异常后锁定，清空历史、撤销 mock 命令，等待人工重新启用 |

`DISABLED` / `FAULT` 的目标占空比为空，**不是 0% 命令，也不是已执行的停车或刹车**。
本版无法处理真实驱动器失联；真实适配器、ESP32 看门狗和实测降级 PWM 必须一起完成后，
才可接实际电机。停止 ROS 节点同样不等于停止真实电机。

本包只有位置前馈，没有张力、放线长度或电流反馈。两端直线距离是几何间距，
不能视为实际放线长度，不能识别全部松线、打滑、卡线。模拟占空比 0.2 / 0.4
只是展示数值，不是实机推荐值；PWM 也不能直接当作力矩或张力。

## 本机离线验证（不需要 ROS）

使用系统 Python 3.8 及以上版本和标准库，不需要 pip 安装依赖。
从任意目录运行：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/zjh/orange/src/tether_winch_control/src \
python3 -m unittest discover \
  -s /home/zjh/orange/src/tether_winch_control/test -v
```

运行内置手持轨迹模拟，可将全部样本保存到 CSV：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/zjh/orange/src/tether_winch_control/src \
python3 /home/zjh/orange/src/tether_winch_control/scripts/simulate_winch \
  --csv /tmp/tether_winch_trace.csv
```

替换模拟用的两档数值和判定参数：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/zjh/orange/src/tether_winch_control/src \
python3 /home/zjh/orange/src/tether_winch_control/scripts/simulate_winch \
  --low-duty 0.15 --high-duty 0.35 \
  --window-sec 0.2 --enter-speed-mps -0.05 \
  --exit-speed-mps -0.02 --enter-hold-sec 0.15
```

模拟程序覆盖静止预热、远离、靠近、停止、等距离圆弧横移、收放换向、
高档时定位中断、定位恢复但不启用、人工重新启用。它直接使用纯控制核心和 mock 端，
不导入 ROS，也不发送网络数据。命令行参数独立于 ROS YAML，修改 YAML 不影响此模拟。
CSV 包含每个样本的阶段、距离、径向速度、状态、目标占空比、有效性、原因和启用事件；
默认只在终端打印阶段或档位变化。

## 坐标、时序和判定

默认接入 `/mavros/local_position/odom`，要求 `frame_id=map`、`child_frame_id=base_link`。
默认值仅与现有仓库接入习惯一致，实机必须检查。其他定位源需显式修改话题及坐标系配置，
不能自动切换坐标系，也不应只改帧名而不转换数据。

挂线点位置为 `p_attach = p_odom + R(orientation) * attachment_offset`，
距离为 `norm(p_attach - anchor)`。`anchor` 是固定地面出线点在父坐标系中的三维坐标，
`attachment_offset` 是挂线点相对定位 child 原点、按 child 轴表达的偏移（米）。
起飞原点未必等于出线点；默认零偏移必须在手持验证前确认或测量替换。
不读取 `Odometry.twist`；定位姿态用于旋转挂线点偏移。

在源时间戳覆盖的滑动窗口内，以距离做最小二乘直线拟合，斜率就是径向速度。
算法先减去窗口首时间戳，避免大 epoch 时间造成精度丢失。以下参数均在 `config/winch.yaml`：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `low_duty` / `high_duty` | `unset` / `unset` | 未标定时不允许启用；内部约束 `0 <= low < high <= 1` |
| `window_sec` / `min_samples` | 0.2 秒 / 3 | 斜率拟合窗口及最低样本数 |
| `enter_speed_mps` | −0.05 m/s | 低于此值才开始统计连续靠近时间 |
| `enter_hold_sec` | 0.15 秒 | 满足连续靠近后进入高档；中途不满足则重新计时 |
| `exit_speed_mps` | −0.02 m/s | 达到或高于此值时返回低档 |
| `near_anchor_m` | 0.1 米 | 距离不大于此值时禁止高档 |
| `timeout_sec` | 0.5 秒 | 源数据年龄、接收间隔和样本间隔上限 |
| `future_tolerance_sec` | 0.05 秒 | 允许源时间戳略超前当前源时钟的上限 |
| `max_position_speed_mps` | 3 m/s | 手持验证时的定位位移合理性检查值 |
| `position_jump_slack_m` | 0.05 米 | 跳变检查的固定位置容差 |
| `max_samples` | 1000 | 防止异常高频数据无限积累 |

跳变判定上限为 `position_jump_slack_m + max_position_speed_mps * dt`，其中 `dt`
是相邻源时间戳之差。该检查作用于定位原点位置；它不能识别所有小幅定位重置。
过慢靠近可能落入死区而保持低档，这是设计取舍，需纳入地面标定。

窗口及进入确认会带来切换延迟；停止后也要等拟合速度回到退出阈值，
不能把程序输出理解为零延迟电机响应。高档期间若窗口内样本不足，直接故障退出，
不会继续保持高档。启用要求至少 3 个新鲜有效样本，启用初始输出为低档。

源时间戳及 `source_now` 使用同一定位时钟；接收和看门狗使用独立单调时钟。
源时间戳零值、非有限数据、无效四元数、重复/乱序、帧不符、数据超时、
源时钟回退、异常位移都会导致故障。故障后有效数据可以重新积累，
`input_valid=true` 只表示输入恢复，并不清除 `FAULT` 或恢复输出。
必须再次调用启用服务。ROS 适配器用墙钟线程运行看门狗，`/clock` 停止时也检查接收超时。

## ROS 接口（待板端验证）

节点名：`tether_winch_control`，配置由 launch 加载为私有参数。

- 输入：可配置 `odom_topic`，类型 `nav_msgs/Odometry`。
- 启停：`/tether_winch_control/set_enabled`，类型 `std_srvs/SetBool`。
- 状态：`/tether_winch_control/status`，类型 `tether_winch_control/WinchStatus`，默认 20 Hz。
- 状态包含档位、距离、径向速度、目标占空比、定位有效性和原因。
  `command_valid=false` 时 `target_duty=NaN`；无可用距离/速度时对应值也是 `NaN`。
  `hardware_feedback_valid` 始终为 false，`applied_duty` 始终为 NaN，不能把目标值当成实际输出。

先将功能包同步到板端实际工作空间，再在**板端**运行以下命令。本次不在本机执行：

```bash
ssh orange
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build tether_winch_control
source /home/orangepi/orange/devel/setup.bash
roslaunch tether_winch_control tether_winch.launch simulation_values:=true
```

此启动方式仍是 mock，只加载示例两档数值；它不生成定位，需要另有定位节点发布。
实际几何参数仍须修改 `winch.yaml` 或通过 launch 的 `config` 参数选择配置文件。
去掉 `simulation_values:=true` 则采用未标定默认值，拒绝启用。

另一个**板端终端**观察状态、启用和停用：

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
rostopic echo /tether_winch_control/status
```

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
rosservice call /tether_winch_control/set_enabled "data: true"
# 结束验证时执行；mock 停用不代表真实驱动器停车。
rosservice call /tether_winch_control/set_enabled "data: false"
```

板端后续验证还包括消息生成、launch 参数合并、服务行为、节点关闭、时钟暂停、
定位恢复与重启行为。纯 Python 测试通过不能替代这些集成检查。

## 手持地面标定顺序

1. 旋翼动力禁用，定位正常运行；先以 mock 观察远离/靠近/静止/横移是否判定正确。
   记录实际出线点、挂线点偏移以及定位坐标原点；不要依赖开机位置正好重合。
2. 真实通信及失联处理验证完成后，从小输出开始找放线低档：阻力低、不明显顿挫，
   卷筒无失控空转；检查静止时少量余线能否缓慢收完。
3. 从接近低档的值逐步增加收线高档，选择能跟上预期最大靠近速度的最小输出。
   优先检查“靠近→停止→重新拉紧”时是否明显拽手。
4. 重复慢速靠近、快速靠近、突然停止、换向、横移和不同卷筒层数/线长；
   检查持续堵转输出能力和温升。可临时串接测力计记录拉力峰值与波动。
5. 接受标准是在明确测试范围内余线不持续增加、停止后可收完且无明显拉拽，
   静止不反复切档。若低档收不完或高档有冲击，记录两档方案的边界，不盲目加大输出。
6. 地面通过后才做低速飞行对照。手持感觉与模拟轨迹都不能证明“对飞行无影响”。

## ESP32 接入前需要补齐

- 固件/驱动器型号、可读取的接口说明或源码；确认占空比方向和真实力矩关系。
- IP 是否仍为 `192.168.31.87`、实际协议和端口、PWM 频率/分辨率/取值范围。
- 命令确认、实际应用值回报、重复或过期命令处理、连接恢复是否需重新启用。
- 台架验证的停用/故障输出与恢复流程；ESP32 本地超时看门狗，不能依赖 ROS 进程发送最后一帧。
- 两档 PWM、最大手持靠近速度、适用线长/卷筒层数和允许张力的实测记录。

本版没有假设 UDP/HTTP，也没有编造 ESP32 指令；不修改或刷写未知固件。
