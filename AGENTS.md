# Orange 项目工作说明（公共，入 Git）

> 本文件由 macOS、Ubuntu 与板端通过 Git 共享，只记录与机器无关的仓库事实与协作规范。
> 开始操作前，若工作区根目录存在未入库的 `AGENTS.local.md` 或 `.env`（模板见 `.env.example`），请先读取它们以获取当前机器的环境限制、SSH 别名和本地路径。

## 多端角色与平台分工

- **板端（Orange Pi 5 Max）**：RK3588，Ubuntu 20.04.6 LTS，ROS Noetic（`desktop-full`）+ `catkin-tools`（`catkin build` 可用，ROS 位于 `/opt/ros/noetic`），工作空间 `~/catkin_ws`。板端是唯一的实机算法编译、录包、建图与实机定位/飞行验证端；ROS 核心固定在 dummy 网卡 `ros0`。
- **Ubuntu 开发机**：用于底层 RViz 调试（`tools/ego_rviz`、`tools/ego_map_demo`）及原生 ROS 编译/测试。
- **macOS 开发机**：用于代码编辑、Git 协作、离线分析与 `tools/warehouse_monitor` 网页监控（通过 SSH 隧道连接板端，无需配置 ROS 主机路由）。

## 仓库结构

工作空间共有 33 个 ROS 包，默认发现 27 个。目录名保留上游写法，ROS 包名取自各自的 `package.xml`。

| 源码目录 | 包目录 |
|---|---|
| `src/drivers/` | `livox_ros_driver2`、`ov13855_cam`、`ov9281_usb_cam`、`barcode_reader_serial`、`tether_winch_control` |
| `src/localization/` | `FAST_LIO`（包名 `fast_lio`）、`LiDAR_IMU_Init`（包名 `lidar_imu_init`）、`fastlio_gravity_bridge`、`fastlio_dll`、`ywl_localization`、`voxel_slam`、`drone_mapping`、`drone_relocalization`、`car_bridge` |
| `src/navigation/` | `drone_navigation`、`front_marker_control`、`ego_swarm/` 下的 6 个包（`bspline_opt`、`path_searching`、`plan_env`、`plan_manage`〔包名 `ego_planner`〕、`quadrotor_msgs`、`traj_utils`） |
| `src/perception/` | `front_marker_vision`、`ov13855_apriltag_vision`、`barcode_reader_camera`、`barcode_rc_trigger`、`barcode_arrival_trigger` |
| `src/visualization/` | `orange_visualization` |
| `src/experiments/` | `camera_calibration_pkg`、`mid360s_calib`、`dual_camera_recorder`、`vz_chirp_control`、`vz_step_control` |

以下六个包在各自的包根目录带有空文件 `CATKIN_IGNORE`。

- `lidar_imu_init`、`vz_chirp_control`、`vz_step_control` 可分别启用。
- `barcode_reader_serial` 可单独启用。启用 `barcode_rc_trigger` 时还需启用 `barcode_reader_serial`；启用 `barcode_arrival_trigger` 时需要这三个包。OV13855 相机读码包 `barcode_reader_camera` 默认参与构建，室内 test4/test5 可直接使用它。

在 Noetic 工作空间根目录，将所需包及上述带标记依赖的 `CATKIN_IGNORE` 临时改名为 `CATKIN_IGNORE.off`，运行 `scripts/build.sh <ROS包名>`。实验结束后将标记改回原名，使下次包发现恢复默认范围。该标记控制源码发现，已有 devel 产物可能仍存在。

- `scripts/build.sh` 在 Noetic 环境下构建 `third_party/Livox-SDK2` 与 `third_party/gtsam-4.0.3`，安装到工作空间的 `.deps/`，再执行 `catkin build`。 Ubuntu 上的依赖镜像与干净编译、单测命令见 [Noetic 构建验收](docker/noetic-build/README.md)。
- `scripts/` 包含归档配对、常驻服务、室内外导航与离线测试入口。
- `docker/ros-noetic-rviz/` 提供 Ubuntu 和 macOS 的 RViz 镜像及启动器，网络设置见各平台的 `local*.env.example`。
- 实机 EGO 位于 `src/navigation/ego_swarm/`。离线固定版与实机版的维护边界见 [地图演示说明](tools/ego_map_demo/README.md#源码与构建边界)。
- `tools/`：`warehouse_monitor`（仓库巡检网页监控）、`car_web`（车/无人机网页面板）、`ego_rviz`、`ego_map_demo`、`seeker-boot-*`、`front_marker_jitter_analysis.py`、`capture_front_marker_jitter.sh`。

## 入口与本机配置

- 板端使用单一检出。导航和采集入口从自身所在的 Orange 加载 `devel/setup.bash`。移动板端检出后，请从新位置重跑 `scripts/resident/install.sh`。可先运行 `scripts/resident/install.sh --render /tmp/orange-units` 检查生成的两份 systemd 单元。
- 仓库监控通过 SSH 别名 `orange` 连接。不同主机或远端检出路径可在本机 `.env` 设置 `ORANGE_SSH_HOST`、`ORANGE_REMOTE_ROOT`。远端根目录按远端用户解释。
- 独立安装 `seeker-boot-start` 时可建立指向 `tools/` 下脚本的符号链接。复制脚本时，用 `SEEKER_TOOLS` 指向包含生成器与 YAML 的工具目录。
- 小车网页通过相邻的私有 `tools/car_web/config.local.js` 读取多个 rosbridge 端点，例如 `window.CAR_ROSBRIDGE_URLS = ["ws://car-a.local:9090", "ws://car-b.local:9090"];`。这些地址对访问页面的浏览器可见。可用 `?ws=` 单次覆盖，未配置时使用 `ws://spirit.local:9090`。此方式也适用于直接打开 HTML。
- Ubuntu 的快捷入口链接到当前检出，EGO 继续使用对应的 `ego-swarm-rviz` 和 `ego-swarm-map-demo` 用户数据目录。地图和布局位置见 [EGO RViz](tools/ego_rviz/README.md) 和 [离线地图演示](tools/ego_map_demo/README.md)。

### MID-360S

- 将 `src/drivers/livox_ros_driver2/config/MID360s_config.json` 复制为同目录下的 `MID360s_config.local.json`，填写主机网口和雷达 IPv4 地址。驱动与预检默认使用这份私有配置，也可通过 `MID360S_LIVOX_CONFIG` 指定配置文件的绝对路径。
- 板端用 `ORANGE_ROS_MASTER_IP` 核对 `ros0` 的固定地址。该值取自环境变量或工作空间根目录的私有 `.env`。车端 rosbridge 默认使用 `spirit.local:9090`，可通过环境变量 `CAR_HOST` 设置车端主机。

## Git 工作流与安全边界

- 远端：`https://github.com/junheng-sect/Orange.git`，默认分支 `main`。
- 在 `main` 上拉取更新优先使用 `git pull --rebase origin main`；新功能或结构性重构从最新 `main` 切出独立分支，验证后再合并回 `main`。
- 提交前检查 `git status` 与 `git diff`；涉及实机行为的改动以板端跑通为准再合并。
- Commit 信息采用 Conventional Commits 英文 `type` + 简体中文说明：`<type>(<scope>): <中文说明>`（`scope` 可省略，代码名/包名/参数名保留英文），常用 `type` 为 `feat`、`fix`、`docs`、`chore`、`refactor`、`perf`、`test`。
- 严禁向 `AGENTS.md`、`.env.example` 或任何入库文件写入明文密码、私有局域网 IP、MAC 地址或单机个人绝对路径；严禁使用 `git add -f` 提交 `AGENTS.local.md`、`.env` 等已忽略文件。

## 实验主库与跨会话交接

- KIOXIA 移动盘上的 `Log analysis` 是实验主库（各机入口路径见本机 `AGENTS.local.md` / `.env`）：历史板端数据保存在 `log/orange/`，配对实验保存在 `log/YYYY-MM-DD/<实验 ID>/`，分析产物保存在 `outputs/`。复盘优先复用盘上已有副本，联机导入默认使用 `orange` SSH 别名。
- 每次实验复盘在 Obsidian 当天日期目录下单独建立实验文件夹（例如 `<OBSIDIAN_ROOT>/2026/11月/11.11/test8_131708/`），当天日志仅留一行指向实验总结：
  - 在实验文件夹内编写 `总结.md`，记录实验名和日期、bag 与 ULog 路径、图与图例、结论和已知局限。
  - 原图、完整 HTML、绘图脚本与分析数据保存在 `Log analysis/outputs/`；将精选图片和交互 HTML 复制到 Obsidian 实验文件夹，在 `总结.md` 中用 `![[xxx.png]]` 内嵌图，用 `[xxx.html](xxx.html)` 链接交互版。
  - 共享大依赖（如 `plotly.min.js`）放在当天日期目录根，各实验的交互 HTML 用 `../plotly.min.js` 等相对路径引用。Obsidian 文件树默认不显示 `.html` 和 `.js`，从 `总结.md` 打开交互版。
- 开发与分析切换会话时，在实验记录中写明实机版本或未提交改动、地图版本、实验 ID、配对文件、已确认结论和待验证项。联调与飞行使用板端本地记录与地图，归档由电脑拉取。
- 本文件只保存长期有效的仓库架构事实与协作规范，不记录每轮开发流水或一次性操作命令清单。
