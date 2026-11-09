# EGO-Swarm 官方显示，单独接板端真实数据

本工具保留官方白底、无人机模型、蓝色已走轨迹、红色优化控制点连线和膨胀障碍图。
原版布局与显示源码在 `rviz/upstream-default.rviz`、`vendor/`，来源提交固定为
`ZJU-FAST-Lab/ego-planner-swarm@92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`。
校验清单在 `UPSTREAM.json`，上游许可证在 `LICENSE.upstream`。

## 本机使用

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

源码保存在板端 `/home/orangepi/orange/tools/ego_rviz`。本机安装包由该目录同步，
独立 Docker 镜像只构建显示组件，不构建本机独立 ROS 仓库，也不启动规划/控制节点。

```bash
docker build -t ego-swarm-rviz:noetic ~/.local/share/ego-swarm-rviz
```

启动脚本只复用已有 `mid360s_ws/docker/ros-noetic-rviz/scripts/common.sh` 的网络、
GPU与X11检查；不读取或融合它的 RViz 布局。
