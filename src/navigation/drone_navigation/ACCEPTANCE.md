# EGO-Swarm 单机接入：软件验收记录

日期：2026-11-03 12:34:36 Asia/Shanghai。目标：Orange Pi 5 Max / Ubuntu20.04 / ROS Noetic。
原始证据目录：`/home/orangepi/test_data/navigation_acceptance/20261103_123436`。

## 已通过

- 板端 catkin build：导航及依赖7包全部通过；FAST-LIO及Livox依赖2包通过。只构建，没有重启现有硬件节点。
- catkin 测试报告：13 tests，0 errors，0 failures，0 skipped。
- ROS Master 11431 独立集成：同源时间戳输入、非零杆臂、90°航向差和非零平移对齐；P+V掩码2496及ENU数值转换；默认不创建真实setpoint发布者。
- 有序两个不同高度目标、实际模拟里程计到点判定、终点转向、连续任务与旧B样条拒绝。
- 未知/越界/错误地图/占据目标拒绝；带遮挡的1.1m宽墙体阻挡直线后，EGO在允许范围绕行并完成目标。
- 合成场景最大指令合速度：0.3412154713627568 m/s（配置上限0.5m/s）；这是指令，不是实机运动测量。
- action取消、点云断流、恢复不自动续飞、规划器退出，以及单独重启预览网关后的ROS时钟暂停检测。
- 录包31项配置话题均可解析类型；最终4个节点模块可导入，默认enable_output=false且deployment_verified=false；git diff --check通过。
- 额外回归覆盖MAVROS自身target_* / trajectory/desired反馈不造成控制冲突、真实其他控制输入发布者可识别、刚体参考点协方差传播。

## 实现边界

合成模拟器按采样指令理想跟踪，不包含真实PX4动力学、外扰、雷达盲区、玻璃或细杆漏检。
未进行真实bag复算、手持实时点云、飞控闭环或飞行验收；以上不能用于声明真实定位精度或停止距离。
先验地图仍只做启动重定位，未新增持续地图纠漂。路线需要真实地图身份；机体包络、杆臂、出光点与飞行空间必须实测填入并核对。
当前配置是预览示例，真实输出被部署检查禁用。未经实测不得直接设置deployment_verified=true。

## 复现

软件回归：`bash ~/orange/src/navigation/drone_navigation/test/run_isolated.sh`。
单元回归：source ROS和orange/devel后，`catkin run_tests drone_navigation --no-status`。
隔离脚本只起停自己的测试节点与11431端口，不启停常驻roscore/MAVROS、Livox、FAST-LIO、相机或绕线器。
运行和部署参数见README.md；固定上游修订与许可证见../ego_swarm/UPSTREAM.md。


## Review 修复回归（2026-11-03）

证据目录：`/home/orangepi/test_data/navigation_acceptance/20261103_review_fixes.7DaS14`。

- 导航 catkin 构建通过；单元测试23项（原13项＋运行回归10项），0错误、0失败。
- 新增运行回归使用实际 Gateway/Executor 回调、mock ROS传输和时钟：prepare后水平/竖直移动1米，切换OFFBOARD第一帧位置等于切换状态回调时最新PX4位置，速度为零；切换后保持点冻结。
- 新样条校验期间继续旧样条；新候选未知时保留仍有效旧样条；换点等待超过0.3s仍有命令；旧轨迹碰撞、校验过期、取消和跳变检查仍生效；漏FAULT后WAIT_TARGET会停止未完成轨迹，正常终点和初次规划不误停。
- 11431隔离集成完整通过：连续不同高度航点与终点转向、墙体绕行、取消、断云与恢复不续飞、规划器退出、暂停ROS时钟；合成最大命令速度0.34116377541239357m/s。
- Header.seq由rospy单一目标发布者填写，Gateway/Executor取同一已接收序号；双航点集成覆盖了序号序列化后的代次匹配。
- 本轮只构建并运行独立Master的合成预览，不启停硬件或发送真实设定值。未知空间策略未改，手持预览中的unknown footprint次数尚未统计。

- FAST-LIO板端catkin构建通过（fast_lio及Livox依赖）；保留既有PCL uint16_t弃用警告。
- 录包新增4项测试通过：默认不订阅世界点云、开关启用且不重复订阅、显式自定义清单、缺少导航消息仍可登记原生速度和点云；既有录包策略回归及11432独立Master端到端测试通过，两个bag及latch内容读回正确。
