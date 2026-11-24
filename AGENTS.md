# Orange 项目工作说明（公共，入 Git）

> 本文件由 macOS、Ubuntu 与板端通过 Git 共享，只记录与机器无关的仓库事实与协作规范。
> 开始操作前，若工作区根目录存在未入库的 `AGENTS.local.md` 或 `.env`（模板见 `.env.example`），请先读取它们以获取当前机器的环境限制、SSH 别名和本地路径。

## 多端角色与平台分工

- **板端（Orange Pi 5 Max）**：RK3588，Ubuntu 20.04.6 LTS，ROS Noetic（`desktop-full`）+ `catkin-tools`（`catkin build` 可用，ROS 位于 `/opt/ros/noetic`），工作空间 `/home/orangepi/orange`。板端是唯一的实机算法编译、录包、建图与实机定位/飞行验证端；ROS 核心固定在 dummy 网卡 `ros0`（`10.250.32.1/32`）。
- **Ubuntu 开发机**：用于底层 RViz 调试（`tools/ego_rviz`、`tools/ego_map_demo`）及原生 ROS 编译/测试。
- **macOS 开发机**：用于代码编辑、Git 协作、离线分析与 `tools/warehouse_monitor` 网页监控（通过 SSH 隧道连接板端，无需配置 ROS 主机路由）。

## 仓库结构

- `src/`：18 个一级目录（共 23 个 ROS 包）——`drone_navigation`、`ego_swarm`（含 `bspline_opt`、`path_searching`、`plan_env`、`plan_manage`、`quadrotor_msgs`、`traj_utils` 6 个子包）、`fastlio_dll`、`dual_camera_recorder`、`front_marker_control`、`front_marker_vision`、`ov13855_cam`、`ov13855_apriltag_vision`、`ov9281_usb_cam`、`camera_calibration_pkg`、`tether_winch_control`、`vz_chirp_control`、`vz_step_control`、`orange_visualization`、`barcode_*` 四件套（`barcode_arrival_trigger`、`barcode_rc_trigger`、`barcode_reader_camera`、`barcode_reader_serial`）。
- `tools/`：`warehouse_monitor`（仓库巡检网页监控）、`car_web`（车/无人机网页面板）、`ego_rviz`、`ego_map_demo`、`seeker-boot-*`、`front_marker_jitter_analysis.py`、`capture_front_marker_jitter.sh`。

## Git 工作流与安全边界

- 远端：`https://github.com/junheng-sect/Orange.git`，默认分支 `main`。
- 在 `main` 上拉取更新优先使用 `git pull --rebase origin main`；新功能或结构性重构从最新 `main` 切出独立分支，验证后再合并回 `main`。
- 提交前检查 `git status` 与 `git diff`；涉及实机行为的改动以板端跑通为准再合并。
- 严禁向 `AGENTS.md`、`.env.example` 或任何入库文件写入明文密码、私有局域网 IP、MAC 地址或单机个人绝对路径；严禁使用 `git add -f` 提交 `AGENTS.local.md`、`.env` 等已忽略文件。

## 实验主库与跨会话交接

- KIOXIA 移动盘上的 `Log analysis` 是实验主库（各机入口路径见本机 `AGENTS.local.md` / `.env`）：历史板端数据保存在 `log/orange/`，配对实验保存在 `log/YYYY-MM-DD/<实验 ID>/`，分析产物保存在 `outputs/`。复盘优先复用盘上已有副本，联机导入默认使用 `orange` SSH 别名。
- 每次实验复盘在 Obsidian 当天日期目录下单独建立实验文件夹（例如 `<OBSIDIAN_ROOT>/2026/11月/11.11/test8_131708/`），当天日志仅留一行指向实验总结：
  - 在实验文件夹内编写 `总结.md`，记录实验名和日期、bag 与 ULog 路径、图与图例、结论和已知局限。
  - 原图、完整 HTML、绘图脚本与分析数据保存在 `Log analysis/outputs/`；将精选图片和交互 HTML 复制到 Obsidian 实验文件夹，在 `总结.md` 中用 `![[xxx.png]]` 内嵌图，用 `[xxx.html](xxx.html)` 链接交互版。
  - 共享大依赖（如 `plotly.min.js`）放在当天日期目录根，各实验的交互 HTML 用 `../plotly.min.js` 等相对路径引用。Obsidian 文件树默认不显示 `.html` 和 `.js`，从 `总结.md` 打开交互版。
- 开发与分析切换会话时，在实验记录中写明实机版本或未提交改动、地图版本、实验 ID、配对文件、已确认结论和待验证项。联调与飞行使用板端本地记录与地图，归档由电脑拉取。
- 本文件只保存长期有效的仓库架构事实与协作规范，不记录每轮开发流水或一次性操作命令清单。
