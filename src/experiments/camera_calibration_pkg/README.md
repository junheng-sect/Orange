# camera_calibration_pkg

OV13855 单目内参标定（ROS Noetic）。由本机 ROS2 版 `~/project/pi_ws/src/camera_calibration_pkg` 改写：
相机换成 `ov13855_cam`，标定用官方 `camera_calibration/cameracalibrator.py`，经 `auto_save_calibrator.py` 包装：
CALIBRATE 完成后自动打印重投影误差并保存，结果先另存，人工确认后再替换。

## 标定板

`size` 为**内角点数**（列x行），格子数各多 1；`square` 为单格边长（米）。launch 默认 `7x5`、`0.041`，按手上的板修改。

- `board/checkerboard_7x5_41mm_A3.pdf`：默认参数的 1:1 PDF，8x6 格 328x246 mm，A4 放不下，需 A3 打印。
- `board/ros_standard_8x6_108mm.pdf`：旧包附带的 `标定板.pdf`，实为 ROS 标准板，8x6 内角点、108 mm，页面 889x1118 mm，需大幅面打印。

生成其它规格（自动选 A4/A3；打印选 100% / 实际大小，打印后用尺复核边长并以实测值作为 `square`）：

```bash
source /opt/ros/noetic/setup.bash && source /home/orangepi/orange/devel/setup.bash
cd ~/orange/src/experiments/camera_calibration_pkg/board
rosrun camera_calibration_pkg generate_checkerboard.py --cols 7 --rows 5 --square 0.041
```

## 标定

标定窗口是 OpenCV GUI，需要板端桌面。用 NoMachine 连板端，在桌面里打开终端执行
（只有 ssh 时先 `export DISPLAY=:0`，窗口出现在板端桌面，NoMachine 里可见）：

```bash
source /opt/ros/noetic/setup.bash && source /home/orangepi/orange/devel/setup.bash
roslaunch camera_calibration_pkg ov13855_calibration.launch size:=7x5 square:=0.041
```

`stream:=full`（默认）标定主通路 4224x3136，`stream:=self` 标定自通路 1280x960。两路是同一画幅的整幅缩放，
标一路后用 `scale_calibration.py` 换算另一路（见「换算到另一分辨率」），推荐标 4224x3136 再换算 1280x960。
launch 只起被标定那一路的相机节点（`ov13855_full_cam` / `ov13855_cam`），先确认没有其它进程占用相机；
相机已在运行时加 `start_camera:=false`。窗口显示的是缩到约 VGA 的画面，标定用原图角点。

1. 移动标定板到一个新位置后**停稳约半秒**再换下一个位置，只有停住时才会采样（终端出现 `*** Added sample N`）。
   需要覆盖：左右到画面边缘（X）、上下到边缘（Y）、远近（Size）、俯仰/左右倾斜约 30°（Skew），整块板都要在画面内。
   终端提示“新位置未采样：标定板移动 x px/帧”说明停得不够稳；仍很难采样可调大 `max_chessboard_speed`
   （单位为原图像素/帧，默认按 2560 宽 3.0 换算：full 5.0、self 1.5；旧包的 0.5 是 640x480 下的值）。
2. 等 X / Y / Size / Skew 四个条基本变绿、样本 30~60 组后点 `CALIBRATE`。2560x1440 下计算需要几十秒（4224x3136 更久），
   之后还要打包样本图，窗口会暂时无响应，等终端出现“已保存”。
3. 终端依次输出：官方打印的 `D/K/R/P` 和 oST 格式结果（`camera matrix`、`distortion` 等），
   接着是本包追加的重投影 RMS（总体及最差 5 张），最后自动保存并打印新旧 fx/fy/cx/cy 对比。
   RMS 一般应在 1 px 以内；个别图偏大说明该样本模糊或角点错，可重标。
4. 自动保存内容（不用再点 `SAVE`）：
   - `~/.ros/camera_info/<camera_name>_<时间戳>.yaml`（如 `ov13855_4224x3136_...`）：文件头注释写明日期、样本数、棋盘规格、RMS；
   - 同名 `.tar.gz`：样本图 + ost.yaml，另有 `/tmp/calibrationdata.tar.gz`（官方 SAVE 的同一份）。
   另存目录可用 `out_dir:=...` 修改。
5. **不要点 `COMMIT`**：它经相机节点的 `set_camera_info` 直接覆盖 git 跟踪的
   `ov13855_cam/config/<camera_name>.yaml`。

## 离线重算（推荐作为最终结果）

官方工具不能剔除坏样本（例如个别角点误检），对已保存的 tar 重算：

```bash
source /opt/ros/noetic/setup.bash && source /home/orangepi/orange/devel/setup.bash
rosrun camera_calibration_pkg recalibrate.py --size 7x5 --square 0.0397 --max-image-error 1.0
```

默认读取 `~/.ros/camera_info` 下最新的 `ov13855_4224x3136_*.tar.gz`（`--camera-name` 换前缀，`--tar` 可指定），在原图上重新检测角点，
用 3 项径向畸变（`--k 2` 可改）标定，单张 RMS 大于 `--max-image-error`（默认 0.5 px）时每次剔除最差的一张后重算，
至少保留 `--min-images`（默认 15）张。默认 0.5 px 是按 2560 宽定的，4224x3136 像素尺度约大 1.7~2 倍，用 `1.0`；
用 0.5 会一路剔到 15 张下限。原图检测偶有误检（整板角点挤成一团，误差数 px），会被当成最差样本剔掉。
终端打印各参数的标准差，结果存为 `..._<时间戳>_k3.yaml`，文件头注释写明来源 tar、使用张数、RMS 和被剔除的图。

launch 已默认 `-k 3`（官方默认只估 k1、k2，本镜头画面边缘误差明显），`k_coefficients:=2` 可改回。

## 替换包内标定

确认无误后手动替换并重启相机节点：

```bash
cp ~/.ros/camera_info/ov13855_4224x3136_<时间戳>_k3.yaml ~/orange/src/drivers/ov13855_cam/config/ov13855_4224x3136.yaml
```

非 4:3 输出（如 16:9）时 ISP 横纵缩放倍数不同，fx 与 fy 本就不相等，因此不加 `--fix-aspect-ratio`。

自动保存失败时（终端会提示），在窗口点 `SAVE` 后手动保存；也可对已有 tar 重新检测角点算逐图误差：

```bash
rosrun camera_calibration_pkg save_calibration.py --size 7x5 --square 0.041
```

`recalibrate.py`、`save_calibration.py` 默认按主通路 4224x3136；标的是自通路时加
`--camera-name ov13855_1280x960`（`save_calibration.py` 还要加 `--width 1280 --height 960`）。

## 换算到另一分辨率

各档输出都是整幅 4224x3136 缩放、不裁剪，同一镜头只差横/纵缩放比例：K、P 的第一行乘 W1/W0、第二行乘 H1/H0，
主点按像素中心 (c+0.5)·s−0.5，畸变系数不变。替换 4224x3136 标定后，同步换算自通路 1280x960：

```bash
cd ~/orange/src/drivers/ov13855_cam/config
rosrun camera_calibration_pkg scale_calibration.py ov13855_4224x3136.yaml --width 1280 --height 960 \
  --camera-name ov13855_1280x960 --out ov13855_1280x960.yaml
```

输出文件头注释写明来源、比例并保留源文件注释。当前包内 4224x3136 为 2026-10-15 主通路直接标定（23/27 张，RMS 0.59 px），
1280x960 由它换算；此前两份都由 2026-10-11 的 2560x1440 标定换算，与本次差值在标定误差内（fx/fy 差 3~4 px，主点差 3~5 px）。

## 测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/orange
catkin build camera_calibration_pkg && catkin run_tests camera_calibration_pkg --no-deps
```
