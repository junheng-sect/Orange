# seeker 工作说明

## 项目环境

- 工作空间：`/home/zjh/project/seeker`
- 当前模块：`SEEKER_OMNI`
- 本机系统：Ubuntu 22.04
- 本机sudo密码[REDACTED]
- SEEKER 板端：RK3588，Ubuntu 20.04，ROS Noetic
- 板端 seeker 工作空间：`/home/vslam/seeker`

## 两端分工与同步（必读）

本仓库同时存在于两端，通过 GitHub (junheng-sect/Orange) 同步，同步方向固定为单向：

- **SEEKER 板端**（`/home/vslam/seeker`）：唯一开发端。代码修改、commit、push、catkin 构建、实机运行都在这里进行。
- **laptop 端**（`/home/zjh/project/seeker`）：只读查看端。仅用于阅读代码和分析问题，不构建、不运行 ROS 节点、**不修改文件、不 commit、不 push**。
- 更新方式：板端 push 后，laptop 执行 `git pull`。laptop 端任何改动都会造成两端分叉，一律不允许。
- 涉及实机验证的结论，以板端代码为准；laptop 看到的可能落后于板端未提交的修改。

## SEEKER 连接与凭据

- 用户：`vslam`
- SSH 密码：`[REDACTED]`
- `su`/root 密码：`[REDACTED]`

当前已验证的网络信息：

- USB RNDIS：PC 通常为 `10.42.0.1`，SEEKER 为 `10.42.0.2`，作为默认调试链路

## 协作与日志

- `AGENTS.md` 只保存长期有效的环境参数、连接凭据和协作规范，不记录每轮开发流水或操作命令清单。
- 日志统一写入 Obsidian 笔记目录：`/home/zjh/note/Note/<月.日>/LOG.md`。
- 日期目录必须使用不补零格式，例如 `6.12`，不要写成 `06.12`。
- 若当天目录或 `LOG.md` 不存在，先创建再记录。
- 每轮对话如果修改了文件，写入“修改记录”；如果只是排查或回答问题，写入“问题记录”。
- 不再默认向工作空间根目录追加 `LOG.md` 或 `PI_LOG.md`。

## 开发约定

- 优先按教程和板端现有 ROS Noetic/catkin 环境调试。
- 本机和 SEEKER 板端构建 catkin 工作空间时，统一使用 `catkin build`，不使用 `catkin_make`。
- 涉及 launch 或 ROS 节点启动时，优先给出可直接复制执行的完整命令。
- 板端相机相关功能通常需要 root 权限，普通调试优先使用 `vslam` 用户。

### catkin_ws_vins 工作空间红线（只读，禁止重建）

- /home/vslam/catkin_ws_vins 是 seeker.service 开机 bringup 依赖的 VINS/robot_upstart 工作空间，**按只读对待**：禁止在其中增删改文件，禁止任何会触发 configure/build 的命令（catkin_make、cmake、atomic_configure 等）。
- 需要改 autostart 时只改 src/seeker_utils/ 下脚本，不要为"生效"而对该空间跑 catkin_make（脚本可从源码路径直接运行/被上层 launch 引用）。
- 原因：catkin_make 会把生成时环境的 CMAKE_PREFIX_PATH 烤进 devel/_setup_util.py；若 shell 未先 source /opt/ros/noetic/setup.bash，生成脚本不再链接 /opt/ros/noetic，seeker.service 里 rosrun/roslaunch 找不到 → 整机 bringup 失败（2026-09-06 已发生）。
- 万一确需重建（必须用户明确授权）：先 source /opt/ros/noetic/setup.bash 再执行；构建后自检 source ~/catkin_ws_vins/devel/setup.bash && command -v rosrun 命中 /opt/ros/noetic/bin/rosrun；并 sudo systemctl daemon-reload。
- 若发现 devel/_setup_util.py 里 CMAKE_PREFIX_PATH = r''.split(';')（underlay 为空），说明工作空间被错误重建，立即报告，不要继续依赖其构建产物。
