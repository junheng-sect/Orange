# rk_ws 工作说明

## 项目环境

- 工作空间：`/home/zjh/project/orange`
- 当前模块：香橙派5MAX 16G
- 本机系统：Ubuntu 22.04
- 本机sudo密码[REDACTED]
- 板端：RK3588，Ubuntu 20.04，ROS Noetic

## 香橙派 连接与凭据

- 局域网wifi：Xiaomi 密码：[REDACTED]
- 用户：`orangepi`
- SSH 密码：`[REDACTED]`
- `su`/root 密码：`[REDACTED]`

## 香橙派调试手册

- /home/zjh/project/orange/OrangePi_5_Max_RK3588_用户手册_v1.3.pdf

## 协作与日志

- `AGENTS.md` 只保存长期有效的环境参数、连接凭据和协作规范，不记录每轮开发流水或操作命令清单。
- 日志统一写入 Obsidian 笔记目录：`/home/zjh/note/Note/<月.日>/LOG.md`。
- 日期目录必须使用不补零格式，例如 `6.12`，不要写成 `06.12`。
- 若当天目录或 `LOG.md` 不存在，先创建再记录。
- 每轮对话如果修改了文件，写入“修改记录”；如果只是排查或回答问题，写入“问题记录”。
- 不再默认向工作空间根目录追加 `LOG.md` 或 `PI_LOG.md`。
