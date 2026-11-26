# drone_navigation：EGO-Swarm 单机航点导航

在当前 Orange 检出中执行 `export ORANGE_ROOT="$(git rev-parse --show-toplevel)"`，下文命令以此定位仓库。

ROS Noetic / Orange Pi 5 Max。规划坐标为 `drone_odom`，保存航点为 `drone_map`，最终输出转换到 `px4_map`。
默认只发布 `/drone_nav/setpoint_preview`；不会创建 `/mavros/setpoint_raw/local` 发布者。
人工起飞、人工切 OFFBOARD；程序不调用解锁、模式、起飞或降落服务。首版不接系留和视觉控制。

## 构建（板端）

```bash
source /opt/ros/noetic/setup.bash
cd "$ORANGE_ROOT"
catkin build fast_lio --no-status -j2 -p1
source devel/setup.bash
cd "$ORANGE_ROOT"
catkin build drone_navigation ego_planner --no-status -j2 -p2
source devel/setup.bash --extend
source "$ORANGE_ROOT"/scripts/ros_net.sh
```

FAST-LIO 仅增加同源时间戳 `/fastlio/velocity`（camera_init 世界系速度），原滤波器、/Odometry、重力桥和外部视觉定位不变。
源文件写入和重新编译不会替换正在运行的进程；在地面结束旧定位运行后，再由人工启动新二进制。

## 配置与预览

复制 `config/navigation.yaml` 与 `config/route.example.yaml`，不要把示例点当作实际航线。
`map_id` 必须是重定位 `/drone_reloc/result.map` 返回的完整地图目录，且 confirmed=true。
路线文件中每点为 `[x,y,z,yaw_deg]`，全部在 drone_map，z 是地图高度，不是相对当前飞机的爬升量。
部署配置中的 bounds 是当次 drone_odom 下允许飞行的中心位置范围；需要每次重启核对地图关系。

部署必填：`lever_body`（雷达 IMU 到 PX4 控制参考点，FLU）、`sensor_offset_imu`（激光发射点）、
`body_half_extent`（以控制参考点为中心的机体半包络）、`clearance`、bounds、map_id。
`deployment_verified=false` 为示例/预览状态；真实输出启动会拒绝该配置。
EGO 半径与半高由 launch 的 `radius`、`half_height` 设置，必须至少覆盖实测包络加 clearance；真实输出启动会交叉检查。
max_vel/max_acc 默认均为三维合量 0.5，主 launch 可显式传入；EGO 单轴参数保守取合量/√3，执行器逐周期按三维范数检查。

板端分别开终端（各终端均 source 上述环境）：

```bash
cd "$ORANGE_ROOT"/scripts
./fusion.sh --no-winch --no-camera
# 定位已启动后另一个终端；不再起小车桥：
roslaunch drone_relocalization relocalize.launch car:=false map_dir:=/实际地图目录
# 导航（只预览）：
roslaunch drone_navigation navigation.launch config:=/实际配置.yaml
```

等 `/drone_nav/input_status` READY、对齐采集至少 2 秒后：

```bash
rosservice call /drone_nav/prepare '{}'
rosrun drone_navigation run_route /实际路线.yaml
```

预览任务仍需要真实输入或隔离合成输入；静止放在地上不会“完成”飞行航点。
RViz 单点发到 `/drone_nav/goal`，PoseStamped 的 frame_id 必须为 drone_map，姿态为目标 yaw；
2D Goal 工具必须显式设置高度，默认拒绝 z=0 的二维点击，避免误以为是保持当前高度。

真实输出只在另行完成地面与仿真验收、核验飞控 OFFBOARD 丢失策略后启用：
`navigation.launch enable_output:=true config:=...`。先 prepare 并保持至少 1 秒设定值预发送，再人工进入 OFFBOARD，最后明确发送路线。等待 OFFBOARD 时每拍跟随当前 PX4 位置和航向、速度置零；收到 OFFBOARD 切换状态时冻结保持点。
检测到其他 MAVROS 设定值输入话题发布者（target_*、trajectory/desired 等 MAVROS 自身反馈不计入）会拒绝准备或停止导航；不要同时启动 front_marker_control、vz_step/chirp 等运动节点。

## 行为与接口

- `/drone_nav/execute_route`：ExecuteRoute action；输入 map_id 和 PoseStamped[]，反馈 waypoint_index/state，结果 success/reason。
- `/drone_nav/planner_goal`：任务层变换后的 EGO 三维目标，禁止直接用它绕过地图身份检查。
- `/drone_nav/odom`：drone_odom → drone_nav_base（控制参考点），twist 为标准机体系速度和角速度；位姿协方差传播到补偿后的参考点。速度旁路不提供速度协方差，导航 Odometry 只供规划使用，不作为额外融合源。
- `/drone_nav/sensor_odom`：同时间戳激光发射点，专供射线起点；`/drone_nav/cloud` 为实际变换后的世界系点云。
- `/drone_nav/bspline`：EGO 原生 traj_utils/Bspline；`/drone_nav/command` 是采样后的带代次位置/速度/加速度/yaw。
- `/drone_nav/cancel`：std_msgs/Empty，取消规划和执行；action cancel 同样生效。
- `/drone_nav/arrived`：std_msgs/Empty，任务层判到点后和 cancel 一起发。网关据此悬停在最后发出的轨迹终点（即目标），而不是判到点那一刻飞机所在的位置；其他原因停止（故障、action 取消、健康检查失败）仍按当前位置和速度减速保持。
- prepare 锁定本轮坐标对齐；对齐残差超过 0.15m/3°即故障。不会在线漂移转换来移动目标。
- `/drone_nav/land`：要求 READY（OFFBOARD）且本轮在地面 `prepare_takeoff` 记录过参考。先按 `~land_vel`（默认 0.1 m/s，0<值≤0.5）保持 XY/yaw 慢降；低于参考上方15 cm、竖直速度≤0.20 m/s、水平速度≤0.25 m/s连续3 s后进入 `PRESSING`，采用0.35 m/s Z速度下压、忽略Z位置闭环，最长5 s。低/慢条件只允许下压，不当成触地。只有新鲜PX4着地确认触发 `TOUCHED`，冻结最后目标、速度清零，然后每秒最多一次普通 `/mavros/cmd/arming(value=False)`，非阻塞且不重叠；最长5 s未上锁则STOPPED并提示。真实已着地且已上锁后才为 `DONE`。从不发value=True，不用force disarm。原地面下10 cm指令位置限幅保留；到位置限幅后仍以慢降速度接近接地条件，最多2 s，不能清零下降速度；Z速度控制期间该位置字段仅为诊断，实测达到地面下10 cm立即停止并按实测位置保持。下压时速度异常、定位/反馈失效或飞手切出OFFBOARD均停止，不续降、不切AUTO.LAND。
- 平移保持段起始航向，终点停稳后按 10°/s 转到目标 yaw；0.15m、0.1m/s、5°内持续 1s判到点。
- 未知/占据/越界目标拒绝执行；射线地图保留占据证据，只有实际自由射线降低其占据概率。检查整条剩余轨迹：`body_half_extent` 机体框内必须已观测为空闲；外层到 radius/half_height 的余量只拒绝观测到的占据（膨胀层），未知不拒绝——室外头顶开阔天空没有回波，余量壳顶层会一直是未知（2026-11-05 实飞回放：+0.3m 层未知导致执行器 3s 超时）。
- 室外显式启用 `allow_unknown_above` 时，未知格仅在同列向下首先遇到原始占据证据为空闲时放行；下方原始障碍或整列未观测仍阻挡。支持格附近的地面/障碍膨胀不向上无限投影，目标当前位置的膨胀碰撞检查保留。室内默认关闭此例外。
- 点云/状态/规划心跳超时、时钟回退、退出 OFFBOARD、action取消都会清旧轨迹。输入恢复不自动恢复故障任务，需 prepare 和新任务。
- 导航故障且 PX4 估计仍有效时生成减速保持；这不是对突现障碍的独立物理制动保证。PX4 定位无效时停止依赖其位置的输出。
- PX4 的 `/local_position/odom` 速度为机体系；导航坐标对齐包含同步参考点补偿。`FRAME_LOCAL_NED` 消息填 ENU，由 MAVROS 转换，不能再次手动交换坐标。
- 执行轨迹时设定值为位置 + 速度 + 加速度前馈（type_mask 2048，只忽略 yaw rate），加速度取 EGO 样条二阶导；保持 / 刹停流仍为位置 + 速度（2496）。

局部地图是有界栅格，默认20×20×6m/0.1m（上限600万格）；大仓库应拆分已验证路线范围，当前不做地图滑动和通道图搜索。
普通整路线预检仍要求目标已观测到；不会飞入未知区域去探索。启动重定位仍为一次性，局部避障不解决长期绝对漂移。
传感器盲区、玻璃、细杆、动态障碍、制动距离和实测CPU负载均需现场验收。

## 验证与证据边界

```bash
source /opt/ros/noetic/setup.bash
source "$ORANGE_ROOT"/devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s "$ORANGE_ROOT"/src/navigation/drone_navigation/test -p 'test_*.py' -v
```

隔离集成测试（脚本自行起停专用 Master 和测试节点，11431 被占用时拒绝运行）：

```bash
bash "$ORANGE_ROOT"/src/navigation/drone_navigation/test/run_isolated.sh
```

`test/integration.py` 只允许指向隔离 ROS Master（建议11431），由测试包装脚本启动导航后注入合成位姿/点云/FCU状态。
它不启动飞控、雷达、相机或绕线器，不向 MAVROS 的真实 setpoint 话题发布。
结果只证明数学、ROS接口和合成场景行为，不是实机避障/飞行验收。

现有 RC 录包登记导航点云、速度、B样条、状态、目标、预览与实际设定值及TF。原始世界系 `/cloud_registered` 由 `record_registered_cloud` 单独控制，默认关闭；需要时在 rc_record.yaml 启用或用 `RC_RECORD_REGISTERED_CLOUD=true` 覆盖。`/drone_nav/cloud` 仍在导航录制清单中。
固定上游版本和适配范围见 `../ego_swarm/UPSTREAM.md`；上游示例launch不作为实机入口。

低速 EGO 适配初值：控制点间距0.2m、碰撞权重10、可行性权重1、重分配轨迹保持权重10、已膨胀障碍外软距离0.15m。以上仅经合成场景验证，不能直接视为现场参数验收。

执行器保留当前已校验样条和待校验样条；新样条通过后才切换，校验期间继续采样仍通过时效、碰撞和连续性检查的旧样条。
换航点时可继续保持前一段终点，新的航向只在当前目标对应的样条到点后调整；新目标仍受3秒规划等待限制。
运行中意外返回 WAIT_TARGET 会取消执行；初次等待规划与 EGO 正常提前最多10ms完成的终点边界允许通过。

手持预览待验收：保持未知空间拒绝策略，统计 `unknown footprint` 拒绝次数并结合爬升段和雷达可见区域检查。本轮没有开展实机预览。

`test10` 按阶段使用上述检查：地面要求起飞点可用，并拒绝已知占据/越界的后续目标；仅未观测到的后续目标可以待核验。每段出发前必须由当前实时地图证明该目标包络可用，否则保持悬停、有限等待后终止，不向该目标飞行。已知膨胀碰撞优先于未知原因报告，避免待核验逻辑掩盖障碍。

降落状态额外记录 `land_target_z`（位置字段）、`land_ground_z`、`land_limit_z`、`land_limit_reached`、`land_z_velocity_only`（为true时Z位置字段不参与控制）、`land_command_vz`、`land_disarm_requests`、`land_disarm_result`。状态为 DESCENDING/PRESSING/TOUCHED/DONE/STOPPED；降落及等待上锁时独占设定值。服务ACK不算上锁，仍以新鲜实际FCU状态确认。

末段参考 [land_with_tracking_v2](https://github.com/junheng-sect/ROS2PUBLIC/blob/rasip_pi/src/land_with_tracking_v2/land_with_tracking_v2/land_with_tracking_v2_node.py)，保留本项目原0.1 m/s慢降、XY/yaw保持及故障停止。参考项目的0.35 m/s也是下降速度指令，不是直接最低油门；本项目仅在PX4确认着地后普通上锁，未采用其低高度启发式直接请求上锁。
独立ROS验证：`bash "$ORANGE_ROOT"/src/navigation/drone_navigation/test/run_landing_isolated.sh /tmp/landing_check`，只用私有Master与模拟上锁服务，不输出实机设定值。
