# vz_step_control

在当前 Orange 检出中执行 `export ORANGE_ROOT="$(git rev-parse --show-toplevel)"`，下文命令以此定位仓库。

Z 轴硬阶跃响应测试。节点在 OFFBOARD 下保持锁存的 XY 位置和 yaw，只向 Z 注入
可配置幅值与时长的速度阶跃，阶跃之间用完整位置定点（含有限 Z 目标）衔接。

默认流程（重复 `cycle_count` 轮）：

```
定点 stabilize_duration_sec 秒
→ 上升 +step_speed_mps 持续 step_duration_sec 秒
→ 定点 hold_duration_sec 秒
→ 下降 -step_speed_mps 持续 step_duration_sec 秒
→ 定点 hold_duration_sec 秒
→ COMPLETE（继续定点直到退出 OFFBOARD）
```

默认参数 `0.5 m/s × 1 s、定点 3 s、保持 10 s、2 轮` 的控制流程共 47 s；
示例 `0.25 m/s × 2 s` 共 51 s。速度段是真正的硬阶跃：不加速度斜坡或滤波，
PX4 内部 `trajectory_setpoint.position[2]` 仅在速度段为空，位置段始终有限。

## 运行

`front_marker_control`、`vz_chirp_control` 和其他 `/mavros/setpoint_raw/local`
发布节点必须先关闭（节点检测到第二个发布者会拒绝进入测试并 ABORT）。
飞机先在普通位置模式下稳定悬停，然后运行：

```bash
source /opt/ros/noetic/setup.bash
source "$ORANGE_ROOT"/devel/setup.bash
roslaunch vz_step_control vz_step_control.launch
```

等待状态进入 `PRESTREAM`（节点以 100 Hz 预发当前位置定点）：

```bash
rostopic echo /vz_step/status
```

随后由操作者解锁、起飞并手动切入 OFFBOARD。节点锁定当时的 XYZ/yaw，rosbag
同时开始记录；稳定 3 s 且频率达到 90 Hz 后自动开始第一轮阶跃。完成后保持
定点，操作者退出 OFFBOARD 或 disarm 时 bag 立即关闭；同一次 OFFBOARD 内
完成后不会自动重跑，再次进入 OFFBOARD 会生成新测试和新 bag。

当前 bag 路径：

```bash
rostopic echo -n 1 /vz_step/bag_path
```

默认保存目录为 `~/bags`，文件名 `vz_step_000001.bag` 递增；序号由
`.vz_step_sequence`（flock 并发保护）与已有 `.bag/.bag.active` 共同决定，
绝不覆盖。完成或 ABORT 后 bag 再记录 1 s 尾段后关闭。

## 参数入口

控制器与记录器的运行参数统一在 `config/vz_step.yaml` 中维护（阶跃 profile、
遥测门禁、安全包线、输出开关、话题名、记录目录，含中文注释和单位），修改后重启
launch 生效；launch 不再透传调参参数。控制器与记录器读取同一包配置中的公共值
（话题名等），两处不会失配。`record` 仍是 launch 的结构开关，决定是否启动
记录器，并由它派生控制器的 `recorder_required`。要临时换用自定义配置：

```bash
roslaunch vz_step_control vz_step_control.launch \
  config_file:=/tmp/my_vz_step.yaml
```

- `step_speed_mps`：阶跃幅值，上升取 `+`，下降取 `-`，默认 0.2。
- `step_duration_sec`：阶跃时长，默认 3.0。
- `stabilize_duration_sec`：OFFBOARD 进入后的定点稳定期，默认 3.0。
- `hold_duration_sec`：阶跃之间的定点保持时长，默认 10.0。
- `cycle_count`：上升/下降轮数，默认 2。
- `setpoint_rate_hz`：setpoint 发布频率，默认 100。

参数必须有限且满足：速度 > 0、时长 > 0、轮数 ≥ 1，且

- `step_speed_mps < maximum_absolute_vz_mps`（继承 0.60）；
- `step_speed_mps × step_duration_sec < maximum_vertical_span_m`（继承 0.80，
  即上升方向的安全余量；下降回到进入高度，向下欠冲由
  `maximum_vertical_offset_m`（继承 0.45）在运行时保护）。

违规时节点拒绝启动并打印具体参数与边界，不静默截断指令。

## 安全边界（继承 vz_chirp.yaml）

- 实际 VZ 超过 0.60 m/s、峰峰高度超过 0.80 m、低于起点 0.45 m、XY 偏离锁存
  目标超过 0.30 m → ABORT 并捕获最近有效位置持续定点。
- odom/velocity 停更 0.10 s、FCU state 停更 2.5 s → ABORT。
- 速度段与位置段的 `PositionTarget.type_mask` 都忽略三轴加速度（含
  `IGNORE_AFZ`），PX4 不会收到“有限 0”的 Z 加速度而被误判加速度有效；速度段只
  携带 Z 速度阶跃，位置段是带有限 Z 目标的完整定点。
- `LOCAL_POSITION_NED(32)` 与 `POSITION_TARGET_LOCAL_NED(85)` 在每次 OFFBOARD
  进入时同步请求 100 Hz，会话结束（退出 OFFBOARD 或 disarm）和节点关闭时恢复
  30/10 Hz；测试进入 COMPLETE 或 ABORT 后若仍停在 OFFBOARD，在 1 s 录包尾段
  结束后也会恢复 30/10 Hz，下一次 OFFBOARD session 再重新请求 100 Hz。
  稳定期结束时 setpoint/target/velocity 实测频率低于 90 Hz 直接 ABORT，测试中
  低于 90 Hz 持续 1 s ABORT。
- MAVLink 断连只会中止当前 session：重连后若直接回到 OFFBOARD，控制器保持
  ABORT 定点、不会自动重开新测试；只有收到一次“连接正常且 mode != OFFBOARD”
  的状态后，下一次 OFFBOARD 上升沿才会启动新测试。
- `/mavros/setpoint_raw/local` 出现第二个发布者 → ABORT（PRESTREAM 中发现则
  拒绝进入 OFFBOARD）。
- 节点在 OFFBOARD 中重启（或崩溃后 respawn）不会自动开始新测试：只捕获当前
  位置持续定点悬停（ABORT），需要退出并重新进入 OFFBOARD 才会开始新测试；
  期间 setpoint 保持发布。XY 安全边界始终以 OFFBOARD 进入点为参考，捕获
  位置不会重置累积漂移的计量。
- 录包开启时（`record=true`，随 launch 传入 `recorder_required=true`），
  进入 OFFBOARD 若录包器不健康 → ABORT 定点；测试运行中录包器停止/失联/报告
  不健康 → ABORT 定点。测试绝不允许无记录执行。

## 记录内容

录包节点以 respawn 方式运行，录包故障不会影响 OFFBOARD 控制（控制器仍持续
定点，只是中止测试）；崩溃后自动重启，可继续为下一次 OFFBOARD 会话录包。
录包器每秒 2 次在 `/vz_step/recorder_health`（`VzRecorderHealth`，latch）上
发布健康状态，供控制器裁决。

录包器与控制器采用相同的“重连锁定”规则：只有先观察到“连接正常且
mode != OFFBOARD”的状态，才允许下一次 OFFBOARD 上升沿开包。若录包器在飞机
已处于 OFFBOARD 时重启（或崩溃 respawn），它不会自动生成半程新 bag——先保持
不健康等待，待一次真正的退出再进入后才开始记录。

- `/mavros/setpoint_raw/local`：ROS 发出的 setpoint
- `/mavros/setpoint_raw/target_local`：PX4 目标回传代理量
- `/mavros/local_position/velocity_local`：实际本地速度
- `/mavros/local_position/pose`、`/mavros/local_position/odom`：位置姿态
- `/mavros/state`：模式、连接和解锁状态
- `/vz_step/status`：阶段、轮次、指令 VZ、锁存位置、当前 Z/VZ、消息频率、
  异常原因
- `/vz_step/recorder_health`：录包器健康（healthy/recording/bag_path），
  记录中（OFFBOARD 且有 open bag）healthy=true；COMPLETE/ABORT 尾段关闭后仍停
  留在 OFFBOARD 时视为 healthy（不再有阶跃）；OFFBOARD 中无 bag（重启中/
  开包失败）时 healthy=false

## 开机自启动

板端总启动文件 `/etc/ros/noetic/seeker.d/seeker_autostart.launch` 提供：

- `enable_vz_step_control`：是否开机启动本包（当前默认 true）。
- `vz_step_record`：是否启动 OFFBOARD 自动 rosbag。
- `enable_vz_chirp_control`：扫频包开关（当前默认 false）。
- `enable_front_marker_tracking`：Front Marker 总开关。

是否发布真实 setpoint 不再由总启动文件控制，改在
`config/vz_step.yaml` 的 `enable_setpoint_output` 中维护（修改后重启生效）。

三个包都会发布 `/mavros/setpoint_raw/local`，不得同时开启。

## 构建和测试

```bash
cd "$ORANGE_ROOT"
catkin build vz_step_control --no-deps
catkin run_tests vz_step_control --no-deps
catkin_test_results build/vz_step_control
python3 src/experiments/vz_step_control/test/test_sequence_allocation.py
python3 src/experiments/vz_step_control/test/test_recorder_session.py
```

无 FCU 冒烟联调（bench）：

```bash
roscore &
python3 src/experiments/vz_step_control/test/fake_mavros.py --prestream 4 &
roslaunch vz_step_control vz_step_control.launch record:=false
```

可选的实时联调脚本（需要已 build 并 source devel，会自动起 roscore/节点/录包并
自清理，退出码 0 即全部断言通过）：

```bash
# 在真实节点线上验证 type_mask 含 IGNORE_AFZ，以及 COMPLETE 后仍在 OFFBOARD 时
# MAVLink 频率自动恢复 30/10 Hz
python3 src/experiments/vz_step_control/test/integration_vz_wire.py
# 验证录包器被杀后控制器 ABORT 定点、不执行无记录阶跃，且频率在 ABORT 后恢复
python3 src/experiments/vz_step_control/test/integration_recorder_abort.py /tmp/vz_live_bags
```
