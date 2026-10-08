# ov13855_apriltag_vision

OV13855 独立 ROS Noetic AprilTag 视觉包。输入图像与 CameraInfo，输出相机坐标系下的标签位姿。
当前位姿解算直接采用 `Drone-Vision/vision_core/src/vision_detect.cpp` 的默认 OpenCV `solvePnP` 路径，
不依赖旧视觉包，不再包含从旧视觉包迁入的位姿筛选与确认机制。

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
roslaunch ov13855_apriltag_vision ov13855_apriltag_vision.launch
```

## 实际处理流程

1. 将图像转换为灰度；检测输入最长边缩至 1280，使用原生 AprilTag 检测。
2. 角点按实际 X/Y 缩放比例还原至原图，保留 AprilTag 原生 `p[0..3]` 顺序。
3. 默认选择面积最大的标签，也可配置固定 ID。
4. 使用物体角点 `(-h,-h,0)、(h,-h,0)、(h,h,0)、(-h,h,0)`，其中 h=码边长/2，
   与原生 AprilTag 的 BL/BR/TR/TL 角点对应；不按画面方向重排角点。
5. 直接调用 `cv::solvePnP(object_points, corners, K, D, rvec, tvec)`：
   默认 `SOLVEPNP_ITERATIVE`，不使用历史初值，也不再调用额外精修函数。
6. 解算成功且位姿数值有限、相机深度大于零时，将 Rodrigues 向量转为旋转矩阵与四元数，
   直接发布该帧 Pose、TF 和矩阵。首帧、切换 ID、丢失后重现均不等待连续确认。

没有额外的轮廓/边缘对比度检查、重投影阈值、IPPE 双解筛选或历史分支确认。
CameraInfo 必须有效，图像尺寸和 frame 必须与其匹配；无目标、求解失败、非有限结果或非正深度时不发布。
相机到机体的变换继续由控制链的静态 TF 处理，不在视觉节点内重复应用。

## 配置与接口

参数入口：`config/ov13855_apriltag_vision.yaml`，可通过 `config_file:=/absolute/path/vision.yaml` 覆盖。
`layout_file:=` 非空时在其后再加载标签版式文件（`barcode_reader_camera/config/layouts/*.yaml`），覆盖
`target_marker_size_m`；`bringup.launch` 和 `ov13855_marker_tracking.launch` 用 `layout:=<名字>` 传入。

- 原生 AprilTag `tag36h11`，配置的有效黑框边长为 **0.0437 m**（2026-10-08 现场标签实物测量），不含外侧白边/标签纸；
  这是不带版式启动时的默认值，带版式时以版式文件为准；节点代码中的兜底默认值仍为 0.012 m。
- `selection_mode: largest`：全图最大面积；同面积选择较小 ID。
  `selection_mode: fixed`：仅使用 `target_marker_id`，默认值 0。
- 检测器默认最长边 1280、quad_decimate=1.5、4 线程、refine_edges=true。
- 输入 `/ov13855/image_raw`、`/ov13855/camera_info`；mono8/rgb8/bgr8。
- 输出 `/front_marker/pose`：geometry_msgs/PoseStamped，语义 `^header.frame_id T_marker`，继承源图像时间戳。
- 同时间戳发布 `pose.header.frame_id -> front_marker` 动态 TF。
- 调试前缀 `/ov13855_apriltag_vision/`：`status`、`image_annotated`（默认 5Hz、有订阅者时生成）、
  `debug/processing_time_ms`、`debug/homogeneous_transform`。
  矩阵消息为本包 `HomogeneousTransform`，marker_id 为实际选中 ID，matrix 为行主序。
- 成功状态为 `DETECTED: ID… xyz_camera=(…)m pipeline=SOLVEPNP_ITERATIVE`。
  保留缺少标定时的 `DETECTED_2D`，求解失败为 `SOLVEPNP_FAILED`；不再输出参考状态或分支计数。

标签坐标原点为中心，X 沿标签右向、Y 沿标签上向、Z 朝标签正面，正对相机时标签 Z 与光学 Z 相反。
旧的 pose_solver、ippe_*、pose_continuity_*、max_reprojection_error_px、enable_contour_gate 和轮廓参数已删除；
外部自定义配置也应移除这些无效项，本包不提供旧解算模式回退。

## 综合入口与旧链路

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
roslaunch front_marker_control ov13855_marker_tracking.launch
```

外参使用控制包 `config/ov13855_camera_tf.yaml`，子帧为 `ov13855_optical_frame`；控制算法和参数保持现状。
退出新入口后，可用 `roslaunch front_marker_control front_marker_tracking.launch` 启动旧 OV9281 链路。
两个入口不能同时运行，因为共用 `/front_marker/pose` 和 `front_marker` TF。

## 构建和验证

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build ov13855_apriltag_vision
catkin run_tests ov13855_apriltag_vision --no-deps
source /home/orangepi/orange/devel/setup.bash
rostest front_marker_control test_ov13855_input.test
# 相机空闲时运行；独立 master、仅运行相机与视觉，22 秒后退出。
python3 /home/orangepi/orange/src/ov13855_apriltag_vision/test/live_camera_check.py
```

测试涵盖与 Drone-Vision 原始 solvePnP 调用的数值对照、标签四方向姿态、缩放还原、
固定/最大面积选择、首帧及 ID 切换立即发布、基本无效输入停发、Pose/TF/矩阵一致和控制输入外参。
合成测试还确认：只要解算结果符合基本有效性条件，不会因为重投影 RMS 大于 1 px 而拒绝。

参考内参仍按 2560×1440 使用，但原 JSON 没有标定分辨率记录。
当前软件是否发布位姿与实物尺度是否准确是两项验证，最新结果见 `VALIDATION.md`。
