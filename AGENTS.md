# Orange 项目工作说明（公共，入 Git）

> 本文件由 macOS、Ubuntu 与板端通过 Git 共享，只记录与机器无关的仓库约束与协作规范。
> 开始操作前，若工作区根目录存在未入库的 `AGENTS.local.md` 或 `.env`（模板见 `.env.example`），请先读取它们以获取当前机器的环境限制、SSH 别名和本地路径。

## 多端角色与平台分工

- **板端（Orange Pi 5 Max）**：RK3588，Ubuntu 20.04.6 LTS，ROS Noetic（`desktop-full`）+ `catkin-tools`（`/opt/ros/noetic`），工作空间 `~/catkin_ws`。唯一的实机算法编译、录包、建图与飞行验证端；ROS 核心固定在 dummy 网卡 `ros0`。
- **Ubuntu 开发机**：底层 RViz 调试（`tools/ego_rviz`、`tools/ego_map_demo`）与 Docker / 原生 ROS 编译测试。
- **macOS 开发机**：代码编辑、Git 协作、离线分析与 `tools/warehouse_monitor` 网页监控（通过 SSH 隧道连接板端，无需配置 ROS 主机路由）。

## 目录与构建约束

- **目录布局**：ROS 包按功能分布在 `src/{drivers,localization,navigation,perception,visualization,experiments}`，其余顶层目录为 `scripts/`、`tools/`、`docker/`、`third_party/`；模块职责见 [README.md](README.md#目录结构)，具体接口以各包目录的 `README.md` 与 `package.xml` 为准。
- **目录名与 ROS 包名不一致的例外**（调用 `scripts/build.sh <ROS包名>` 时须传包名）：
  - `src/localization/FAST_LIO` → `fast_lio`
  - `src/localization/LiDAR_IMU_Init` → `lidar_imu_init`
  - `src/navigation/ego_swarm/plan_manage` → `ego_planner`
- **按需包（`CATKIN_IGNORE`）**：运行 `find src -name CATKIN_IGNORE` 可查看当前未参与默认发现的包。临时启用时，将目标包及其按需依赖的 `CATKIN_IGNORE` 改名为 `CATKIN_IGNORE.off`，执行 `scripts/build.sh <ROS包名>`，实验结束后恢复原名。该标记仅控制源码发现，已有 `devel` 产物可能仍存在。
  - 跨包依赖：`barcode_rc_trigger` 需同时启用 `barcode_reader_serial`；`barcode_arrival_trigger` 需同时启用上述两个包。室内 test4/test5 直接使用默认启用的 `barcode_reader_camera`。
- **构建与离线验收**：`scripts/build.sh` 先构建 `third_party/`（Livox-SDK2、GTSAM 4.0.3）到 `.deps/` 再执行 `catkin build`。Ubuntu 依赖镜像与干净编译、单测命令见 [Noetic 构建验收](docker/noetic-build/README.md)。
- **双份 EGO 维护边界**：实机规划位于 `src/navigation/ego_swarm/`，`tools/ego_map_demo/official/` 是独立构建的离线固定快照，勿混改；边界见 [离线地图演示](tools/ego_map_demo/README.md#源码与构建边界)。

## 入口与本机配置

- **板端单一检出与常驻服务**：导航和采集入口从所在仓库加载 `devel/setup.bash`。移动板端检出后，须从新位置重跑 `scripts/resident/install.sh`（可先运行 `scripts/resident/install.sh --render /tmp/orange-units` 检查两份 systemd 单元）。
- **SSH 与远端路径**：仓库监控等工具默认使用 SSH 别名 `orange`，可在本机 `.env` 通过 `ORANGE_SSH_HOST`、`ORANGE_REMOTE_ROOT`（按远端用户解释）覆盖。
- **MID-360S 与网络参数**：
  - 将 `src/drivers/livox_ros_driver2/config/MID360s_config.json` 复制为同目录私有文件 `MID360s_config.local.json`，填写主机网口和雷达 IPv4 地址；也可用 `MID360S_LIVOX_CONFIG` 指定配置文件绝对路径。
  - 板端通过环境变量或本机 `.env` 中的 `ORANGE_ROS_MASTER_IP` 核对 `ros0` 固定地址；车端 rosbridge 默认 `spirit.local:9090`，可用 `CAR_HOST` 覆盖。
- **单工具配置**：[car_web](tools/car_web/README.md)、[ego_rviz](tools/ego_rviz/README.md)、[ego_map_demo](tools/ego_map_demo/README.md)、[ros-noetic-rviz](docker/ros-noetic-rviz/README.md) 与 `tools/seeker-boot-start` 等单工具私有配置与数据目录约定见各自文档或脚本头注释。

## Git 工作流与安全边界

- 远端：`https://github.com/junheng-sect/Orange.git`，默认分支 `main`。
- 在 `main` 上拉取更新优先使用 `git pull --rebase origin main`；新功能或结构性重构从最新 `main` 切出独立分支，验证后再合并回 `main`。
- 提交前检查 `git status` 与 `git diff`；涉及实机行为的改动以板端跑通为准再合并。
- Commit 信息采用 Conventional Commits 英文 `type` + 简体中文说明：`<type>(<scope>): <中文说明>`（`scope` 可省略，代码名/包名/参数名保留英文），常用 `type` 为 `feat`、`fix`、`docs`、`chore`、`refactor`、`perf`、`test`。
- 严禁向 `AGENTS.md`、`.env.example` 或任何入库文件写入明文密码、私有局域网 IP、MAC 地址或单机个人绝对路径；严禁使用 `git add -f` 提交 `AGENTS.local.md`、`.env` 等已忽略文件。

## 实验主库与跨会话交接

- KIOXIA 移动盘上的 `Log analysis` 是实验主库（各机路径见本机 `AGENTS.local.md` / `.env`）：历史板端数据保存在 `log/orange/`，配对实验保存在 `log/YYYY-MM-DD/<实验 ID>/`，分析产物保存在 `outputs/`。复盘优先复用盘上已有副本，联机导入默认使用 `orange` SSH 别名。
- 实验复盘在 Obsidian 当天日期目录下单独建立实验文件夹（例如 `<OBSIDIAN_ROOT>/2026/11月/11.11/test8_131708/`），当天日志仅留一行指向实验总结：
  - 在实验文件夹内编写 `总结.md`，记录实验名和日期、bag 与 ULog 路径、图与图例、结论和已知局限。
  - 原图、完整 HTML、绘图脚本与分析数据保存在 `Log analysis/outputs/`；将精选图片和交互 HTML 复制到 Obsidian 实验文件夹，在 `总结.md` 中用 `![[xxx.png]]` 内嵌图，用 `[xxx.html](xxx.html)` 链接交互版。
  - 共享大依赖（如 `plotly.min.js`）放在当天日期目录根，各实验的交互 HTML 用 `../plotly.min.js` 引用（Obsidian 文件树默认不显示 `.html` 和 `.js`，从 `总结.md` 打开交互版）。
- 开发与分析切换会话时，在实验记录中写明实机版本或未提交改动、地图版本、实验 ID、配对文件、已确认结论和待验证项。联调与飞行使用板端本地记录与地图，归档由电脑拉取。
- 本文件只保存长期有效的仓库架构约束与协作规范，不记录每轮开发流水或一次性操作命令清单。
