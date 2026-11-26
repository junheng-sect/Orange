# EGO-Swarm 官方显示，单独接板端真实数据

本工具保留官方白底、无人机模型、蓝色已走轨迹、红色优化控制点连线和膨胀障碍图。
原版布局与显示源码在 `rviz/upstream-default.rviz`、`vendor/`，来源提交固定为
`ZJU-FAST-Lab/ego-planner-swarm@92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`。
校验清单在 `UPSTREAM.json`，上游许可证在 `LICENSE.upstream`。

## Apple Silicon Mac

Ubuntu 使用 `ego-rviz` / `scripts/run.sh`。
Mac 使用新的 `ego-rviz-mac` / `scripts/run_mac.sh`，构建使用 `Dockerfile.macos`。

先安装 Docker Desktop 4.34+，启用 host networking，并从 Orange 根目录构建 Mac 显示镜像：

```bash
./docker/ros-noetic-rviz/scripts/build_image_mac.sh
./tools/ego_rviz/scripts/build_image_mac.sh
./tools/ego_rviz/scripts/run_mac.sh
./tools/ego_rviz/scripts/run_mac.sh --no-map
./tools/ego_rviz/scripts/run_mac.sh --check
./tools/ego_rviz/scripts/run_mac.sh --stop
./tools/ego_rviz/scripts/run_mac.sh --dry-run
```

Mac 镜像为 `ego-swarm-rviz:noetic-arm64-web`，显示组件在容器内构建，默认浏览器地址
`http://localhost:6081/vnc.html`。完整 RViz、模型、路径和地图沿用原来的实现，使用 Mesa 软件渲染。
不在板端添加显示／桥接进程，不默认降采样；原始 PCD 只读挂载并在 Mac 容器加载。
有真实 `drone_map -> drone_odom` 后才能叠加地图与实时无人机，仍不提供恒等变换兜底。
Mac 私人布局仅首次调整窗口尺寸，保留后续保存结果。关闭 RViz 窗口或 `--stop` 会停止全部辅助服务；
关闭浏览器标签页不会停止容器。

共享脚本默认来自本检出的 `docker/ros-noetic-rviz/`。单独安装时用 `EGO_RVIZ_SHARED` 指定该目录。
`--master`／`--ros-master`、`--ros-ip`、`--web-port`、`--name` 可覆盖默认值。
`--no-open` 可仅启动显示容器，不自动打开浏览器。
`--check` 会按 `--odom` 指定的话题实际接收数据并检查 TF，缺少必需数据返回非零。
启动前的本机双向 TCP 探测不连接板端；板端回调与性能仍须在同网时实测。
`--dry-run` 不操作 Docker、路由或个人文件。
本机验证记录在 [Mac 验证记录](../../docker/ros-noetic-rviz/MAC_VALIDATION.md)。
截至 2026-11-09，Mac 尚无原始地图缓存；默认打开实时视图。
拿到原始 PCD 后使用 `ego-rviz-mac --map /本机/地图目录`，不需要在板端新增进程。

本机 `ego-rviz-mac` 已链接到 `scripts/run_mac.sh`，支持相同的 Mac 参数。
Mac 复用同一检出的 `common_mac.sh` 和 `display_runtime_mac.sh`，只读取共用目录的
`local.mac.env`；Ubuntu 仍使用原 `common.sh` / `local.env`。原 `ego-rviz` 名称不作为 Mac 入口。

## Ubuntu 使用

```bash
ego-rviz
ego-rviz --check
ego-rviz --stop
```

已有默认地图时 `ego-rviz` 自动打开地图视图。当前默认是今天室外建图
`20261105_114605_outdoor`，完整4175519点，不裁剪、不降采样。
地图缓存放在本机 `maps/20261105_114605_outdoor`，`maps/current` 指向默认地图。
用 `ego-rviz --no-map` 打开原实时视图，或 `ego-rviz --map /本机/地图目录` 换图；
切换前先 `ego-rviz --stop`。

地图视图使用 `drone_map` 固定坐标，按高度着色，只读加载原始 `map_world.pcd`。
加载地图不代表规划器已导入它作为碰撞地图，也不启动重定位或导航。
无人机、实时扫描、规划线的叠加等待真实重定位提供 `drone_map -> drone_odom`；
不把保存地图与本轮里程计强行设成同一个坐标系。

默认模型/轨迹输入 `/drone_nav/odom`。只跑 FAST-LIO 时可关闭后改用
`ego-rviz --odom /fastlio/odom_world`，该输入也必须在 drone_odom 坐标系。
板端定位、导航与地图由原有流程启动；本工具只启动本机显示节点与 RViz。
无输入时窗口等待真实数据，不注入模拟数据、不启动板端硬件或导航。

本机显示安装位置 `~/.local/share/ego-swarm-rviz`，专用容器 `rviz_ego_swarm`、
镜像 `ego-swarm-rviz:noetic`，配置 `home/.rviz/ego_swarm_real.rviz`。
现有 FAST-LIO 与重定位 RViz 配置、窗口和容器不参与本工具。
窗口内可保存个人布局；模板与官方原文件仍保留。

## 与原版的必要适配

- 只保留 drone0；规划/建图话题接当前真实导航，背景图使用 /cloud_registered。
- Fixed Frame 为 drone_odom；调整原版屏幕外窗口位置、显示侧栏和三维观察角度。
- 官方 odom_visualization 的模型、路径沿用输入 header.frame_id；不把坐标改名成 world。
- 去掉未使用的仿真 PositionCommand 订阅，避免额外消息依赖。保留官方模型和渲染算法。
- 已走路径最多保留6000点，时间回退清路径；不发布 TF。
- 地图模式仅增加 `drone_map -> ego_swarm_map_origin` 的显示原点恒等 TF，不改变运行坐标关系。
- 观察窗口保留选择、旋转、测量工具；任务目标由现有导航流程发送。

红线与上游相同，为 B 样条控制点连线；不是执行器精确采样曲线。
膨胀图和占据图受板端 visualization_truncate_height（当前1.8m）显示截断影响。
官方 goal_point/global_list 在当前单机目标流程中可能无输出，默认关闭。

## 构建及安装

Ubuntu 快捷入口指向本仓库的 `tools/ego_rviz/scripts/run.sh`，地图、布局和日志位于 `${XDG_DATA_HOME:-$HOME/.local/share}/ego-swarm-rviz`。

独立 Docker 镜像只构建显示组件，不构建本机独立 ROS 仓库，也不启动规划/控制节点。

```bash
docker build -t ego-swarm-rviz:noetic tools/ego_rviz
```

启动脚本只复用已有 `docker/ros-noetic-rviz/scripts/common.sh` 的网络、
GPU与X11检查；不读取或融合它的 RViz 布局。
