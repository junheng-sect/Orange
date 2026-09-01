# rk_ws 工作说明

## 项目环境

- 工作空间：`/home/zjh/project/rk_ws`
- 当前模块：`SEEKER_OMNI`
- 本机系统：Ubuntu 22.04
- 本机sudo密码[REDACTED]
- SEEKER 板端：RK3588，Ubuntu 20.04，ROS Noetic
- 板端 seeker 工作空间：`/home/vslam/seeker`
- 模块教程：`/home/zjh/project/rk_ws/SEEKER_OMNI模块教程 (3).pdf`

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

### 板端直接写日志（pclog）

- 在 SEEKER 板端会话中，无需回 PC 侧补记，直接用 `~/bin/pclog` 写入 laptop 日志。
- 用法：`pclog 修改记录 "内容..."`、`pclog 问题记录 "内容..."`；多行内容可用 `cat xxx | pclog 问题记录`。
- 依赖 `~/.ssh/config` 的 `Host laptop`（10.42.0.1，用户 `zjh`，密钥 `id_ed25519_laptop` 免密），laptop 端公钥已加入 `authorized_keys`。
- 日志格式：脚本自动按 `date +%-m.%-d` 定位/创建 `Note/<月.日>/LOG.md`，追加 `## 类型 (YYYY-MM-DD HH:MM:SS)` 小节。
- laptop 不可达时自动暂存 `~/pclog_offline.md`，恢复后执行 `pclog --replay` 重放。
- 仅当 pclog 不可用（如链路/免密失效）时，才退回“由 PC 侧补记”的方式。

## 开发约定

- 优先按教程和板端现有 ROS Noetic/catkin 环境调试。
- 本机和 SEEKER 板端构建 catkin 工作空间时，统一使用 `catkin build`，不使用 `catkin_make`。
- 涉及 launch 或 ROS 节点启动时，优先给出可直接复制执行的完整命令。
- 板端相机相关功能通常需要 root 权限，普通调试优先使用 `vslam` 用户。
