# drone_inspection

巡检任务使用 `missions/` 内的 YAML，或命令行指定的本地 YAML。在已构建并加载 Noetic 工作空间的板端运行：

```bash
rosrun drone_inspection run_mission --mission missions/indoor_shelf.yaml --dry-run
rosrun drone_inspection run_mission --mission missions/indoor_shelf.yaml --preview
rosrun drone_inspection run_mission --mission missions/indoor_shelf.yaml
```

`--dry-run` 校验 PCD 指纹、先验栅格、地图标定、重定位配置和机体包络，并打印各段航点。需要使用另一个目录里的同一张图时，加 `--map <目录>`。预览启动感知和真实规划，网关关闭飞控输出，使用预览起点作为返航 XY；需要手持跟随或模拟机体反馈才能完成到点。

## 编写任务

以 [室内预设](missions/indoor_shelf.yaml) 为例。换图时一并填写地图 SHA-256、重定位配置、与该图配套的航点和边界。`map_frame` 供 car_bridge 发布公共地图标定，单机仅使用 `drone_map` 时可省略。相对路径相对 YAML，支持 `~` 和 `package://`。将预设放进包的 `missions/` 并在桌面同步对应版本的地图，重启监控后可选择。有动作定义变更时重新选择任务。

| 动作 | 航点字段与行为 |
|---|---|
| `pass` | 仅填 `name`、`xyz`、`action`。连续通行点与下一个停靠点合为一段，中间按导航策略控制航向，末点停稳。|
| `hover` | 指定 `yaw_deg` 和 `hold_s`。到点转至指定朝向，再开始计时。|
| `scan_shelf` | 指定这面货架的 `yaw_deg`。到点先稳定 1 秒，其余 `hold_s` 用于观察，至少观察 2 秒；随后筛选新标签、分批抵近并读取 QR。|

航点和限高属于 `drone_map`。导航边界属于 `drone_odom`，起飞高度相对本次地面。规划速度和加速度使用 `planner_max_vel`、`planner_max_acc`，网关与执行器使用 `max_vel`、`max_acc`。线速度用 m/s，加速度用 m/s²，航向用度。地面启动需要手动在 POSCTL 解锁。配置返航时，以末航点地图高度返回本轮起飞 XY。自动慢降要求地面起飞、返航停稳和降落预检通过。手飞接入使用 `auto_takeoff: false`、`return_home: false`、`auto_land: false`，在空中悬停后由飞手切入 OFFBOARD。

## 记录与收尾

任务与旧导航共用互斥锁和定位链路。飞行结束后关通道 6、着地上锁并切回 POSCTL，再从网页收尾或在 CLI 按 Ctrl+C。任务保存在 `~/test_data/nav_test/`，目录名含开始时间、任务名与运行后缀。bag 和已解析配置直接保存在任务目录，子进程日志、任务快照和事件先写入 tmpfs，正常收尾时归档到同一目录。

- `mission.resolved.yaml`、`navigation.resolved.yaml` 保存本次实际生效的配置。
- `mission_state.json` 记录阶段、点位、动作、终态与原因。`mission_events.jsonl` 追加阶段事件。`scan_results.json` 记录观察批次、候选点与每次扫码结果。三种记录均带格式版本、运行 ID 和任务 YAML 摘要。
- 室内预设默认 `save_frames: 'off'`；可选 `'failed'` 或 `'all'`。观察点 PNG、导航点云录包与 DLL 旁路分别独立配置。

## 隔离验证

在工作空间根目录运行以下脚本，各自创建独立回环 master，使用模拟飞控与相机。

```bash
bash scripts/nav_test/tests/run_outdoor_full_isolated.sh --indoor6 /tmp/inspection-old
bash scripts/nav_test/tests/run_outdoor_full_isolated.sh --inspection /tmp/inspection-new
bash scripts/nav_test/tests/run_outdoor_full_isolated.sh --inspection-modes /tmp/inspection-modes
bash scripts/nav_test/tests/run_indoor4_isolated.sh --inspection /tmp/inspection-preview
bash scripts/nav_test/tests/run_indoor4_isolated.sh --inspection-path /tmp/inspection-path
```

最后两项使用 EGO、执行器和关闭输出的网关。现场验收另行配对 bag、任务记录及飞手下载的 ULog；通过后再迁移旧 test6 入口。
