# vz_chirp_control

用于悬停工作点附近的 Z 速度对数扫频。节点保持进入 OFFBOARD 时的 XY 位置和
yaw，只向 Z 速度注入小幅 chirp。

默认参数：

- setpoint 发布：100 Hz
- 扫频：0.2 Hz -> 8 Hz，60 s
- 幅值：0.2 m/s
- 淡入/淡出：2 s
- 理论指令位移峰峰值：约 0.262 m
- 实际高度保护：相对起点 0.45 m、历史峰峰值 0.80 m

## 运行

`front_marker_control` 和其他 `/mavros/setpoint_raw/local` 发布节点必须先关闭。
飞机先在普通位置模式下稳定悬停，然后运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/vslam/seeker/devel/setup.bash
roslaunch vz_chirp_control vz_chirp_control.launch
```

等待状态进入 `PRESTREAM`：

```bash
rostopic echo /vz_chirp/status
```

随后由操作者切入 OFFBOARD。节点锁定当时的 XY/yaw，rosbag 同时开始记录；稳定
3 秒且三路频率均达到 90 Hz 后自动开始扫频。扫频完成后保持 `vz=0`。操作者退出
OFFBOARD 或 disarm 时 bag 自动关闭。

当前 bag 路径：

```bash
rostopic echo -n 1 /vz_chirp/bag_path
```

默认保存目录为 `/home/vslam/seeker/logs/vz_chirp`。每次进入一次 armed OFFBOARD
都会创建一个新的标准 ROS1 `.bag`。

## 记录内容

- `/mavros/setpoint_raw/local`：ROS 发出的 setpoint
- `/mavros/setpoint_raw/target_local`：PX4 目标回传代理量
- `/mavros/local_position/velocity_local`：实际本地速度
- `/mavros/local_position/odom`：位置和姿态
- `/mavros/state`：模式、连接和解锁状态
- `/vz_chirp/status`：扫频阶段、频率和安全状态

注意：PX4 内部 `trajectory_setpoint` 和 `vehicle_local_position_setpoint` 是 uORB
主题，rosbag 不会把上述 MAVROS 代理量伪装成内部原始主题。

## 预览与参数覆盖

不向 MAVROS 发布真实 setpoint：

```bash
roslaunch vz_chirp_control vz_chirp_control.launch enable_setpoint_output:=false
```

配置入口为 `config/vz_chirp.yaml`。最高频率必须不超过发布频率的十分之一。

## 开机自启动开关

板端总启动文件 `/etc/ros/noetic/seeker.d/seeker_autostart.launch` 提供：

- `enable_vz_chirp_control`：是否开机启动本包。
- `vz_chirp_enable_setpoint_output`：是否发布真实 setpoint。
- `vz_chirp_record`：是否启动 OFFBOARD 自动 rosbag。
- `enable_front_marker_tracking`：Front Marker 总开关。
- `enable_vz_step_control`：Z 阶跃测试包 `vz_step_control` 总开关。

当前部署默认值为：VZ step 包、真实 setpoint 和自动 rosbag 开启，扫频包与
Front Marker 关闭。三个控制包不得同时开启，因为它们都会发布
`/mavros/setpoint_raw/local`。

## 构建和测试

```bash
cd /home/vslam/seeker
catkin build vz_chirp_control --no-deps
catkin run_tests vz_chirp_control
catkin_test_results build/vz_chirp_control
```
