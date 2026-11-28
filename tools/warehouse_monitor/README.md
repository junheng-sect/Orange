# 仓库无人机巡检

在当前 Orange 检出中执行 `export ORANGE_ROOT="$(git rev-parse --show-toplevel)"`，下文命令以此定位仓库。

网页、地图缓存和巡检点库运行在打开浏览器的电脑。实时监看时，该电脑通过 SSH 直接连接 Orange Pi，板端从现有 ROS 节点读取数据。浏览器和板端遥测端口均只监听环回地址，三维视图需要 WebGL 2。

## macOS 直连部署

在 Mac 上运行以下命令。需要 Python 3.11 或更新版本、Git，以及能通过 SSH 公钥登录 Orange Pi 的 Mac 账户。浏览器可用 Safari 或 Chrome。Mac 运行网页和点云处理无需 Ubuntu、Docker 或 ROS。

```bash
git clone --branch orange https://github.com/junheng-sect/Orange.git
cd Orange/tools/warehouse_monitor
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

已有仓库时，在 `orange` 分支执行 `git pull --ff-only origin orange`，再进工具目录。

首次连接使用 SSH 别名 `orange` 核对板端指纹。在线监控和地图同步使用公钥登录。桌面端按 `--host`、`ORANGE_SSH_HOST`、`orange` 的顺序选主机。`ORANGE_REMOTE_ROOT` 默认为板端的 `~/orange`，也支持远端绝对路径或相对于远端家目录的路径。两个变量均可写在当前 Orange 检出根目录的私有 `.env`，进程环境优先。地图来自远端登录账户的 `~/test_data/mapping`。

```bash
ssh -o BatchMode=yes orange true
./warehouse-monitor --host orange --sync 20261114_101654_indoor_fly_raw_000001 20261105_114605_outdoor 20261126_204546_indoor_outdoor_clean_air
./warehouse-monitor --host orange
```

打开 [Mac 的 127.0.0.1:8765](http://127.0.0.1:8765/)，保持启动终端打开，Ctrl+C 退出。同步的地图 ID 应与板端当前使用的地图目录一致。Mac 会单独保存地图缓存和巡检点。离线试用可运行 `./warehouse-monitor --offline`，此时无需连接 Orange Pi。

Mac 的飞控状态通过同一条 SSH 隧道读取。默认使用板端环回端口 `8767`，与 Ubuntu 的 `8765` 区分；Mac 浏览器仍打开本机 `8765`。需要其他端口时使用 `--board-port`。板端需部署同一版本的 `tools/warehouse_monitor`，并从同一检出加载 `docker/ros-noetic-rviz/local_view/scripts/drone_status.py`，其只读飞控检查由板端监看进程复用。

## Ubuntu

图形登录后打开 [本机 127.0.0.1:8765](http://127.0.0.1:8765/)。本机以 `warehouse-monitor.service` 常驻时，可用 `journalctl --user -u warehouse-monitor.service` 查日志。飞控状态沿用本机 `drone-status.service`。Linux 也可以用 `--board-status` 改从板端遥测隧道读取。

两个页签在页面可见时都轮询顶部飞控状态。「实时监看」和正在监看的运行任务按需读取三维遥测。使用板端状态源时，飞控状态轮询也会维持 SSH 隧道；最后一次续租满 15 秒后关闭隧道。断线时页面保留，可继续使用本地地图和巡检功能。定位、地图 TF 与数据时间检查通过后，才叠加位姿、点云和航线。

三维视图用左键拖动旋转、Shift + 左键拖动平移、滚轮缩放。

「显示地图」默认选中室内图（11 月 14 日独立飞扫图），也可选室外图（11 月 5 日）或室内+室外图（11 月 26 日清理空中孤立点版）。每项使用该地图 ID 在本机缓存中的最新版本。

联合图可分别勾选「室内」「室外」，并叠加「地面」「环境结构」筛选。在全图或正上方视角只保留室内时，视图会缩放到起点房间。室内范围来自该次点云中实测的墙面、门口、地板和顶棚，边界附近按 `mixed_map_rooms.json` 的包围盒近似划分。

## OV13855 相机

打开页面后，4:3 画框自动显示 OV13855 相机，在「实时监看」和「预设任务」间切换时可连续观看。自动模式在任务产生角点时叠加同帧结果；选择本地识别可在电脑检查 tag36h11 和 QR。扫码重试和全分辨率读码结果附采集时间显示。浏览器切后台或关闭页面后，观看租约最长五秒到期。

板端使用 RKISP 1280×960/15 Hz 和 MPP H.264；独立 SSH 连接只传编码数据和结构化结果。Ubuntu 电脑的视频功能依赖系统 Python 的 OpenCV（含 aruco）、NumPy、PyGObject、GStreamer 的 h264parse 和 avdec_h264。安装对应组件后重启本机监看服务。扫码存图在电脑归档阶段绘制。

## 读码情况

读码面板默认跟随板端最新的室内 test4、test5、test6，显示实验时间（北京时间）、整趟任务状态和各观察点的 Tag、QR 文本、读码结果。下拉选择实验可回看历史，选「跟随最新」恢复自动切换。飞行途中或结束后打开网页都能读取已落盘的结果，板端终端启动的任务也会出现在这里。

电脑通过只读 SSH 获取板端任务目录中的目标记录、流程记录和相机诊断。成功帧按相机帧从 1 开始显示。任务确认读码成功但缺少匹配的相机记录时，帧号显示为「—」。跳过、未尝试和中断单独显示。页面在前台时约每两秒刷新，连接中断时显示上次取得的数据和时间。

## 巡检点与路线预览

选一张本机缓存地图即可离线编辑。常驻服务外也可用 `warehouse-monitor --offline --port 8767` 启动独立离线窗口。在线和离线使用同一份巡检点库，按地图版本分别保存到 `~/.cache/warehouse-monitor/inspection/`。

1. 选择显示地图，打开「巡检点与路线」。新增后输入 Z，再用「地图点选 XY」选通道上的观察位置，也可以直接填 XYZ。观察航向以地图 +X 为 0°，向 +Y 转为正，停留以秒计。
2. 保存、勾选要访问的点，用上下箭头调整顺序。确认巡航 Z，点击「生成路线」。第一站作为预览起点；点位与巡航高度不同，会检查升降段。修改点位或巡航高度后需重新生成。
3. 浅绿色为巡检预览，橙色为板端 EGO 实时轨迹。若某段没有连通通道或端点净空不足，侧栏显示原因，地图保留可预览的部分航段。

预览使用经校验的原始 PCD。按 0.10 m 栅格检查巡航截面，水平预留 0.40 m、垂直预留 0.28 m，并限制在点云覆盖范围内。这是静态几何估计，PCD 没有记录每个空白区域的可见性。预览不调用 EGO 或执行接口，不发送飞行指令。

## 导入新地图或更新版本

地图重建或页面提示「电脑未缓存当前地图版本」时，在电脑上运行：

```bash
warehouse-monitor --sync 20261114_101654_indoor_fly_raw_000001
```

参数是板端 `~/test_data/mapping` 下的地图目录名。已有对应的 `ego-rviz` 本机缓存时，工具校验 SHA-256 后直接复用。缺少匹配的 PCD 时，从板端同步一次。电脑端完成分层降采样，缓存放在 `~/.cache/warehouse-monitor`。同步后刷新网页即可加载新版本。

- 白色机头箭头随姿态更新，优先读取 `/drone_nav/odom`。青色是监看期间的已飞轨迹。
- 紫色是 `/drone_nav/cloud` 的显示采样。橙色是采样后的 EGO B 样条，支持普通目标与连续航段。
- 板端监看启动在 `--telemetry-only` 模式。大地图读取和降采样移到电脑；实时点云发布频率及订阅方式沿用现有监看。

## 前端开发

仓库已带构建后的网页文件，普通部署无需 Node.js。修改前端源码后可在本目录用 esbuild 重新打包：

```bash
npm install --no-save --no-package-lock three@0.180.0 esbuild@0.25.12
./node_modules/.bin/esbuild monitor.js --bundle --format=iife --minify --target=es2020 --outfile=web/monitor.js
```

## 预设任务下发

打开本机巡检页，选显示地图，再到「预设任务」选择脚本和查看航线。

- 室外图 `20261105_114605_outdoor` 提供 test1–12。室内飞扫图 `20261114_101654_indoor_fly_raw_000001` 提供室内 test1–4，采用 `--ev-mode odom`。旧室内图可在折叠的「手工预览」中加载原有点位。
- 「确认任务，等待上电」将请求保存在电脑服务内，最多等待 15 分钟。板端连接后由 `warehouse-nav` tmux 会话等待飞控，确认地面上锁后启动测试。刷新页面或断开 Wi-Fi 不会结束已启动的任务。电脑服务重启会清除未送达请求，板端重启需要重新下发。
- 室外 test6、test10、test11 选中后有独立确认。等「等待遥控解锁」再地面解锁，脚本会切 OFFBOARD 并起飞。带导航的手飞起飞任务按网页提示悬停，等设定值预发送后切 OFFBOARD。
- 室外 test12 全程遥控手飞。原室外重定位确认、FAST-LIO2 与通道 6 录包器就绪后，可拨通道 6，确认录包后静置 20 秒。网页主位姿与已飞轨迹使用 FAST-LIO2 经原重定位对齐到地图的结果。DLL 在旁路匹配先验图，加载、失配和退出只改变对照简表的可用性。落地上锁后关通道 6，确认已封包再点击「结束任务并收尾」。板端可直接执行 `"$ORANGE_ROOT"/scripts/nav_test12.sh`，或加 `--no-dll` 使用原手飞录包入口。
- 室外 **test12 · YWL** 使用同一套手飞和录包流程，地图位姿来自 YWL。板端入口为 `"$ORANGE_ROOT"/scripts/nav_test12.sh --ywl`。YWL 失效时网页回到仍有效的 FAST-LIO2 地图位姿并显示原因。
- 飞行中以遥控器接管。着地上锁后，结束通道 6 录包，再从网页收尾。板端可用 `tmux attach -t warehouse-nav` 查看运行输出，`Ctrl+B D` 分离。日志在板端 `~/test_data/nav_test`，网页启动的目录名称带有请求 ID 前八位。
- 室内 test3 由飞手起飞悬停，设定值预发送后切 OFFBOARD，从当前位置规划到货架左下拍照点。到点后对准货架，按左下、左上、右上、右下拍摄，每点保存一张全分辨率 PNG 和时间戳、位姿。任务只跑一圈，照片和目标记录在同一板端日志目录。现场确认机体及顶部结构后方可实飞。
- 室内 **test3_1 · 四点 AprilTag 识别** 沿用 test3 航线，每点稳住 1 s 后拍一张全图，并在 2 s 窗口统计 tag36h11。每点每个 ID 在 `apriltag_observations.csv` 占一行，保留图像时间戳、命中帧数及代表帧的机体位姿。终点显示 0–5 的覆盖数、缺失和额外 ID。板端入口 `"$ORANGE_ROOT"/scripts/nav_test_indoor3_1.sh`，支持 `--preview`。
- 室内 test4 沿用四个拍照点。每个观察点识别标签后，按本批最短访问顺序到各标签前方 0.8 m（相机光心）读取 QR，最后返回右下悬停。任务栏显示当前标签和读取结果，图上青绿色编号为本批扫码顺序。板端也可运行 `"$ORANGE_ROOT"/scripts/nav_test_indoor4.sh`。
- 室内 test5 用标签四角和固定纸面朝向拟合中心；观察点和扫码点地图航向 −89.5°，光心停距 0.80 m，机体地图限高 1.70 m。
- 通用巡检预设从 `drone_inspection/missions/*.yaml` 读取。网页按缓存地图的 PCD 指纹筛选，板端在下发和启动时核对任务版本。任务状态和扫码历史支持 `mission_state.json` 与 `scan_results.json`。
- 历史测试的 `tasks.json` 由板端 `python3 build_task_catalog.py` 导出，包含真实纳秒时间戳和 PCD SHA-256。修改预设任务或重建任务绑定的地图后，重新导出目录并同步电脑地图缓存。任务的计划航点、EGO 实时轨迹分别绘制。


## test12 的 DLL 对照差

同一雷达 IMU 原点、同一扫描结束时间、同一重定位会话的 `/dll_localization/pose` 减去 `/drone_reloc/pose`。网页给出水平距离、向上为正的高度差和最短有符号航向差。DLL 只发布分析用的 correction 话题，实机地图 TF 由原重定位节点发布。

在板端用实际封包路径导出，第二个参数指定新目录。

```bash
source "$ORANGE_ROOT"/devel/setup.bash
python3 "$ORANGE_ROOT"/tools/warehouse_monitor/export_dll_drift.py "$bag" "${bag%.bag}_drift"
```

`drift.csv` 保留每个局部里程计帧的时间、有效性、原因和两路位置。差值仅在 TRACKING、校正年龄小于 1.5 秒、原定位与 TF 新鲜并有同帧位姿和校正时填写。`summary.json` 记录配对比例、不可用区间、水平差与绝对高度差/航向差的中位数、P95 和最大值。这些值量度相对先验地图的偏离。存在同源裁图清单时，导出器也用原始静止点云做多初值 ICP，单列各算法相对参考的位置误差；可用 `--icp-map 裁图目录` 指定随包地图。缺少参考或配准失效会在报告中说明。

室内 test3_1 和后续使用 `nav_test_main` 的新带图任务同样显示 DLL 对照差。通用任务在 `/nav_test/observer` 宣告本次会话，导航主位姿和轨迹按原有逻辑显示。室内默认用重定位原图与室内参数；在各任务目录用相同导出命令即可读取通道 6 的原名 bag。轻量包不含原始雷达时仍输出完整逐帧差值和覆盖率。`--record-raw-lidar` 供需要重跑、对静止段求 ICP 的采集使用。

- 室内 **test3_5 · 四点减振对照**在任务面板填写本趟硬度（可留空）。先开通道 6，再人工起飞并切 OFFBOARD；右下保持后接管降落，上锁后关通道 6。视频、四点各三张原图、里程计及事件按 bag 归档。预览和评分命令见仓库的 `scripts/nav_test/README.md`。

### 室内原地起降测试

在「室内图」选择「室内原地起降测试」。起飞高度相对本趟起飞位置，默认 0.50 m。连续停稳 2 s 后额外悬停，默认 3 s，再原地慢降。确认框显示本趟三个参数。预检通过后拨通道 6 录包，在 POSCTL 地面人工解锁。确认着地上锁后，切回定点、关通道 6 并在网页收尾。

板端也可运行 `scripts/nav_test_indoor_takeoff_land.sh --height 0.50 --hover 3 --delay 3`。加 `--dry-run` 可离线查看参数。本趟配置、bag、`flight_*.json` 和 `goal_*.json` 位于相同的任务目录。记录中的 `MPC_TKO_RAMP_T` 来自地面只读采样，`null` 表示未读到。
