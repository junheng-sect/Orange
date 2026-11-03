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

软件回归：`bash /home/orangepi/orange/src/drone_navigation/test/run_isolated.sh`。
单元回归：source ROS和orange/devel后，`catkin run_tests drone_navigation --no-status`。
隔离脚本只起停自己的测试节点与11431端口，不启停常驻roscore/MAVROS、Livox、FAST-LIO、相机或绕线器。
运行和部署参数见README.md；固定上游修订与许可证见../ego_swarm/UPSTREAM.md。
