# FAST-LIO2 + DLL

在 ROS Noetic 上为现有 FAST-LIO2 约 10 Hz 的里程计叠加地图纠偏。配准核心采用 [DLL 固定版本](https://github.com/robotics-upo/dll/tree/04623521a21e809f8b695d65a141387fb9b1a2f9) 的距离场与四自由度优化，使用原地图坐标建立 0.10 m 网格。

## 室外同包回放 DLL 与 YWL

早期室外对照子图为 `$HOME/test_data/mapping/20261105_114605_outdoor_shared`，范围包含现有 test11 航点。当前 test12 使用下文的截图裁图。

室外 test12 会一起启动 FAST-LIO2、原室外地图重定位、DLL 和 Channel 6 录包。在网页的室外预设启动，或在电脑上执行：

```bash
ssh -t orange '~/.local/bin/record_outdoor_dll_ywl'
```

板端也可直接运行 `~/mid360s_ws/scripts/nav_test12.sh`。先让 Channel 6 处于关闭位，等原重定位确认且 DLL 进入 TRACKING，再拨到录制位并留够 20 秒静止数据。落地静止几秒后拨回关闭位，约 1 秒后封包。着地并上锁后按 Ctrl+C 收尾，SSH 断开可用 `tmux attach -t nav12` 接回。bag、DLL 日志、地图哈希和运行配置保存在 `~/test_data/nav_test/<时间>_test12/`。原手飞录包入口使用 `--no-dll`。

封包后用 `rosbag info` 确认原始雷达为 `livox_ros_driver2/CustomMsg`，IMU 为 `sensor_msgs/Imu`。PX4 ULog 按原方式下载。

```bash
root="$HOME/test_data/fastlio_dll/20261117"
"$root/outdoor_ws/src/fastlio_dll/scripts/replay_pair_outdoor.sh" \
  /path/to/outdoor_dll_ywl.bag "$root/outdoor_pair_$(date +%Y%m%d_%H%M%S)"
```

入口先用室外配置从同包重跑双窗口初值，再顺序使用独立 ROS master 回放 DLL 和 YWL。两者使用同一份 PCD 和同一个确认时间戳。YWL 用完整姿态做切图，DLL 沿用四自由度配准。若首段静止时间足够但初值窗还需延长，可设置 `PAIR_SEED_SECONDS`，默认 18 秒。正式比较用完整 bag。

### 按截图裁图的 DLL 性能回放

截图选区存放于 `$HOME/test_data/mapping/20261105_114605_outdoor_screenshot_roi`。先把截图的透视边界投影到地图参考地面，再按所得 XY 多边形和原高度带筛选 PCD，保留选中点的原始密度。相机反推和裁图边界分别保存在 `roi_source.json` 和 `manifest.json`。`prepare_outdoor_map.py --xy-polygon` 可复现裁图。

该裁图单独做起飞区粗定位时出现候选歧义。此次离线 DLL 性能回放使用同一历史包在同源原图坐标下已经确认的初值。回放脚本只在初值文件明确声明 `reuse_same_source_map_frame` 且两个子图记录的原图 PCD 及元数据哈希一致时接受这种复用。

```bash
root="$HOME/test_data/fastlio_dll/20261117"
ws="$root/outdoor_ws"
source "$ws/devel/setup.bash"
DLL_REPLAY_CONFIG="$ws/src/fastlio_dll/config/outdoor_large_field.yaml" DLL_STARTUP_TIMEOUT=240 \
  "$ws/src/fastlio_dll/scripts/replay.sh" \
  "$HOME/test_data/mapping/20261105_114605_outdoor/raw.bag" \
  "$HOME/test_data/mapping/20261105_114605_outdoor_screenshot_roi" \
  "$root/screenshot_roi_replay_$(date +%Y%m%d_%H%M%S)" \
  "$root/screenshot_roi_trial/dll_seed.json"
```

10 cm 距离场预算为 1280 MiB。`summary.json` 包含从读取 PCD 到距离场完成的时间、配准耗时分项以及局部里程计帧间隔。收包间隔使用 1× 回放时的 ROS 时钟。

## 实机旁路录包

按原有方式启动 FAST-LIO2、重力桥及同一地图的 `drone_reloc`。在香橙派另开终端，使用本任务的独立工作区。

```bash
ws="$HOME/test_data/fastlio_dll/20261117/observer_ws"
source "$ws/devel/setup.bash"
source "$HOME/mid360s_ws/scripts/ros_net.sh"
rosrun fastlio_dll record_observation.sh \
  "$HOME/test_data/mapping/20261021_110311_indoor" \
  "$HOME/test_data/fastlio_dll/flight_$(date +%Y%m%d_%H%M%S)"
```

录包可以在重定位确认之前启动。起飞前保持静止，用下列命令查看 DLL，等状态变为 `TRACKING`。结束时在录包终端按 Ctrl+C，等待 `observation.bag` 写完，一起保留 `map.sha256` 和 `rosparams.yaml`。原始雷达会占用约 4 MB/s。

```bash
rostopic echo /dll_localization/status
```

`observe.launch` 也可单独启动旁路节点，传入绝对路径 `map_dir:=...`。使用独立裁图时，再用 `legacy_map_dir:=...` 指向原重定位地图。节点在建场前验证裁图清单中的原图 PCD、原图元数据、裁图 PCD 和裁图元数据 SHA-256。旁路只从同一原重定位节点发出的已确认结果、健康状态和匹配的实时 TF 接收初值。它检查初值时间晚于本次重力对齐，固定 TF 重置或原定位失效后清掉旧会话的输出。TF 由原重定位节点发布，DLL 结果通过独立话题录制。

飞行包可用 `python3 scripts/summarize_replay.py <录制目录>` 导出按时间戳匹配的 XYZ 和航向差。原定位与 DLL 使用同一个局部姿态参考点。差值反映两种估计的变化。

## 香橙派回放

把本包复制到独立 catkin 工作区的 `src/fastlio_dll` 下。依赖现有 `mid360s_ws` 已构建的 FAST-LIO、重力桥、重定位包及板端 Ceres、PCL。

```bash
run="$HOME/test_data/fastlio_dll/20261117"
ws="$run/ws"
source /opt/ros/noetic/setup.bash
source "$HOME/mid360s_ws/devel/setup.bash"
cd "$ws"
catkin config --extend "$HOME/mid360s_ws/devel" --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build fastlio_dll -j2
source "$ws/devel/setup.bash"

"$ws/src/fastlio_dll/scripts/replay.sh" \
  "$HOME/test_data/mapping/indoor_fly_raw_000001.bag" \
  "$HOME/test_data/mapping/20261021_110311_indoor" \
  "$run/replay"
```

输出目录须是新目录。默认用回环地址的 ROS master（11471，可用 `DLL_REPLAY_PORT` 指定另一个端口）。脚本只播放 `/livox/lidar` 和 `/livox/imu`，重力桥以 `send_to_mavros=false` 运行。脚本默认调用现有重定位程序计算初值，也可把同一 bag 和地图的重定位 JSON 作为第四个参数复用。

结果包括 `result.bag`、`trajectory.csv`、`candidates.csv` 和 `summary.json`。轨迹 CSV 中的 `fixed_map_*` 用第一条被 DLL 接受的校正投影全程局部轨迹，便于与持续纠偏在同一坐标下比较。输入 SHA-256、初值和日志也保存在结果目录。

验证旁路和录包入口时，在 `replay.sh` 的第一个参数加 `--observe`。它会使用同包同地图的离线初值，在独立 master 上模拟旧重定位接口，注入错地图和旧会话初值，再提交有效结果。bag、统计和验收结果位于输出目录的 `observer/` 子目录。

## 接口与坐标

| 接口 | 数据 |
|---|---|
| 输入 `/Odometry`、`/cloud_registered_body` | 按扫描结束时间精确配对，点云已去畸变，使用雷达 IMU 原点 |
| 输入 `/tf_static` | 复用重力桥发布的 `drone_odom→camera_init` |
| 输入 `/fastlio/odom_world` | 沿用约 10 Hz 本地输出及既有 `drone_base` 语义 |
| 输入 `/dll_localization/seed` | `TransformStamped`，当前局部会话的 `drone_map→drone_odom` 初值，可替换启动初值 |
| 输出 `/dll_localization/pose` | `PoseStamped`，地图坐标下的 `drone_base` 位姿，位置仍为雷达 IMU 原点 |
| 输出 `/dll_localization/correction` 和 `/tf` | `drone_map→drone_odom`，使用校正生效时的本地位姿时间戳 |
| 输出 `/dll_localization/status` | `DiagnosticArray`，候选测量、最后接受和生效时间、质量指标与拒绝原因 |

局部姿态与点云沿用已有安装角和激光—IMU 外参，飞控继续负责现有 EV 杆臂补偿。DLL 以当帧雷达 IMU 为 yaw 旋转支点，只估计位置与航向。局部里程计下一帧起采用已接受的校正。初次定位和恢复需要三帧一致的结果。连续拒绝或校正超龄时先保留上一次校正，进入有限范围搜索。搜索失败后，状态进入 `WAITING_FOR_SEED`。恢复诊断时一并查看校正龄期。

普通回放由 DLL 发布 `drone_map→drone_odom`。旁路模式的 `correction` 只作为对照数据，且初值只来自原重定位。回放使用一组新的节点和 master。
