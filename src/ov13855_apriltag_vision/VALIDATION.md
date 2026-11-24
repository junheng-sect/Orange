# 直接采用 Drone-Vision solvePnP 的验证记录（2026-09-22）

## 查阅归档

原始数据位于 KIOXIA 的 `Log analysis/log/orange/ov13855_apriltag_vision_validation_20260922/`。下文的数据文件名均相对于该归档目录。也可浏览[保护标签中的原始快照](https://github.com/junheng-sect/Orange/tree/pre-merge-20261124/src/ov13855_apriltag_vision/validation)。

归档根目录的 `SHA256SUMS` 包含 285 个文件，其清单文件自身的 SHA-256 为 `aec46442638b1d015c818950876555a7e52e9a659d4a0723ceafe797ca151024`。进入归档目录后，在 macOS 运行 `shasum -a 256 -c SHA256SUMS`，在 Ubuntu 运行 `sha256sum -c SHA256SUMS`，即可逐项复核。

## 当前实现

位姿路径改为参考项目 `Drone-Vision/vision_core/src/vision_detect.cpp` 的默认
`cv::solvePnP(object_points, image_points, K, D, rvec, tvec)`，即 SOLVEPNP_ITERATIVE，
不使用历史初值。原生 AprilTag p[0..3] 对应物体角点
`(-h,-h,0)、(h,-h,0)、(h,h,0)、(-h,h,0)`。

已移除新包之前迁入的轮廓/边缘门禁、IPPE 双解、额外 LM 精修、重投影阈值、
历史参考、连续三帧确认及相关配置、状态字段和测试。旧求解源码和构建目标已删除，
动态链接确认为 `libov13855_pose_solver.so` 和检测库。
保留参考代码的基本求解有效性和 ROS 图像/CameraInfo 一致性检查。

位姿仍为相机到标签变换，Pose/TF/矩阵继承源图像时间戳。
用修改前 SHA256 清单验证：相机包、旧视觉包、旧相机包及控制包文件均未改变。
没有调整默认内参、12 mm 码尺寸、图像参数、外参或控制参数。

## 构建与自动验证

板端 `catkin build ov13855_apriltag_vision --no-deps` 通过，无编译警告。

- 4 项求解器测试通过：与参考项目原始调用数值对照（旋转和带噪角点）、
  已知合成位姿恢复、有限解不因 RMS 大于 1 px 被拒绝、不可用输入拒绝。
- 3 项 AprilTag 测试通过：原生角点顺序、四方向姿态、缩放还原和目标选择。
- 1 项视觉 ROS 集成测试通过：首帧立即发布、ID 切换立即发布、丢失后首帧恢复；
  无标定/无效内参/frame 或尺寸不符/无标签时停发；Pose/TF/矩阵一致。
- 1 项原控制输入集成测试通过：相机外参和左右上下方向保持正确，
  `enable_setpoint_output=false`，未产生 MAVROS setpoint。

共 9 个实际测试用例；catkin 的 XML 聚合统计可能重复计数，不作为用例总数。
旧 IPPE 测试已替换，不以旧结果冒充新路径通过。

## 实机 ID 0 复测

独立 master，约 22 秒，只运行新相机与新视觉节点，采样结束后退出。

| 指标 | 结果 |
| --- | --- |
| 图像 | 2560×1440 mono8，ov13855_optical_frame |
| 接收/处理帧数 | 169 / 169 |
| CameraInfo 同戳配对 | 169/169 |
| 成功位姿与 ID | 169 条，全部 ID 0 |
| 接收 / 源戳频率 | 8.262 / 8.195 Hz |
| 图像接收帧龄 P50 / P95 | 83.53 / 98.57 ms |
| 处理耗时 P50 / P95 | 44.11 / 67.47 ms |
| 状态 | DETECTED: ID0 … pipeline=SOLVEPNP_ITERATIVE |

status 消息会对完全相同的文本限频，因此其条数不能作为识别或位姿条数；
本次位姿和矩阵订阅均统计到 169 条 ID 0 输出。

原始数据见 `direct_solvepnp_results.json`，标注画面见 `direct_solvepnp_detected.jpg`。

## 验证边界

已证实直接求解链路能识别并发布位姿。当前仍使用默认 12 mm 码尺寸及参考内参，
屏幕显示的黑框实际边长没有测量，原 JSON 也没有标定分辨率记录。
输出位置不能直接当作真实距离精度验收；本次没有做尺度标定、运动精度或实飞验证。

实际采集仍约 8.2 Hz，低于配置 caps 的 15 FPS。原控制位姿接收超时仍为 100 ms，
帧间隔可能超过该阈值；此次仅替换求解流程，没有调整该控制行为。

切换前的测试、约 14 px 重投影拒绝及原始采样数据保留在
`previous_pipeline_validation.md` 及同目录历史文件中。

## 分辨率性能测试（2026-09-22）

已完成7档分辨率×15/30 fps×仅相机/相机加检测的28组实测，另有2组前后基线。[性能报告、汇总 CSV 和对比图](https://github.com/junheng-sect/Orange/tree/pre-merge-20261124/src/ov13855_apriltag_vision/validation/performance_20260922_run2) 在归档中分别为 `performance_20260922_run2/` 下的 `REPORT.md`、`summary.csv` 和 `comparison.png`。原始数据、每秒CPU/温度/频率和启动日志保留在同目录；首次监测API错误导致的失败尝试不纳入统计。

在板端重新测试时，确保相机空闲并保持标签和场景不动。脚本默认新建 `~/test_data/ov13855_apriltag_vision/performance_YYYYMMDD_HHMMSS/` 保存结果，也可用第一个位置参数指定一个尚不存在的输出目录。

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
python3 /home/orangepi/orange/src/ov13855_apriltag_vision/test/benchmark_performance.py
```

测试脚本不启动控制节点，最后恢复原相机配置并释放设备；统计脚本test/report_performance.py接收结果目录参数，需要NumPy和Matplotlib。正式相机和视觉配置未修改。
