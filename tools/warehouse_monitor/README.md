# 仓库无人机巡检

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

连接板端所在的局域网。Xiaomi 网络上的板端地址为 `192.168.31.32`，以下以它为例。若 Mac 已经配置好 `ssh orange`，可把下面的目标换成 `orange`。首次连接核对板端 SSH 主机指纹，在线模式和地图同步需能通过 `BatchMode=yes` 登录。

```bash
ssh -o BatchMode=yes orangepi@192.168.31.32 true
./warehouse-monitor --host orangepi@192.168.31.32 --sync 20261021_110311_indoor
./warehouse-monitor --host orangepi@192.168.31.32
```

打开 [Mac 的 127.0.0.1:8765](http://127.0.0.1:8765/)，保持启动终端打开，Ctrl+C 退出。同步的地图 ID 应与板端当前使用的地图目录一致。Mac 会单独保存地图缓存和巡检点。离线试用可运行 `./warehouse-monitor --offline`，此时无需连接 Orange Pi。

Mac 的飞控状态通过同一条 SSH 隧道读取。默认使用板端环回端口 `8767`，与 Ubuntu 的 `8765` 区分；Mac 浏览器仍打开本机 `8765`。需要其他端口时使用 `--board-port`。板端需部署同一版本的 `tools/warehouse_monitor`，并保留现有的 `mid360s_ws/docker/ros-noetic-rviz/local_view/scripts/drone_status.py`，其只读飞控检查由板端监看进程复用。

## Ubuntu

图形登录后打开 [本机 127.0.0.1:8765](http://127.0.0.1:8765/)。本机以 `warehouse-monitor.service` 常驻时，可用 `journalctl --user -u warehouse-monitor.service` 查日志。飞控状态沿用本机 `drone-status.service`。Linux 也可以用 `--board-status` 改从板端遥测隧道读取。

可见页面停在「实时监看」时才启用板端只读遥测。最后一次有效轮询满 15 秒后关闭隧道。断线时页面保留，可继续使用本地地图和巡检功能。定位、地图 TF 与数据时间检查通过后，才叠加位姿、点云和航线。

三维视图用左键拖动旋转、Shift + 左键拖动平移、滚轮缩放。

## 巡检点与路线预览

选一张本机缓存地图即可离线编辑。常驻服务外也可用 `warehouse-monitor --offline --port 8767` 启动独立离线窗口。在线和离线使用同一份巡检点库，按地图版本分别保存到 `~/.cache/warehouse-monitor/inspection/`。

1. 选择显示地图，打开「巡检点与路线」。新增后输入 Z，再用「地图点选 XY」选通道上的观察位置，也可以直接填 XYZ。观察航向以地图 +X 为 0°，向 +Y 转为正，停留以秒计。
2. 保存、勾选要访问的点，用上下箭头调整顺序。确认巡航 Z，点击「生成路线」。第一站作为预览起点；点位与巡航高度不同，会检查升降段。修改点位或巡航高度后需重新生成。
3. 浅绿色为巡检预览，橙色为板端 EGO 实时轨迹。若某段没有连通通道或端点净空不足，侧栏显示原因，地图保留可预览的部分航段。

预览使用经校验的原始 PCD。按 0.10 m 栅格检查巡航截面，水平预留 0.40 m、垂直预留 0.28 m，并限制在点云覆盖范围内。这是静态几何估计，PCD 没有记录每个空白区域的可见性。预览不调用 EGO 或执行接口，不发送飞行指令。

## 导入新地图或更新版本

地图重建或页面提示「电脑未缓存当前地图版本」时，在电脑上运行：

```bash
warehouse-monitor --sync 20261021_110311_indoor
```

参数是板端 `~/test_data/mapping` 下的地图目录名。已有对应的 `ego-rviz` 本机缓存时，工具校验 SHA-256 后直接复用。缺少匹配的 PCD 时，从板端同步一次。电脑端完成分层降采样，缓存放在 `~/.cache/warehouse-monitor`。监看自动按版本选图，等到缓存更新后自动重试。

- 白色机头箭头随姿态更新，优先读取 `/drone_nav/odom`。青色是监看期间的已飞轨迹。
- 紫色是 `/drone_nav/cloud` 的显示采样。橙色是采样后的 EGO B 样条，支持普通目标与连续航段。
- 板端监看启动在 `--telemetry-only` 模式。大地图读取和降采样移到电脑；实时点云发布频率及订阅方式沿用现有监看。

## 前端开发

仓库已带构建后的网页文件，普通部署无需 Node.js。修改前端源码后可在本目录用 esbuild 重新打包：

```bash
npm install --no-save --no-package-lock three@0.180.0 esbuild@0.25.12
./node_modules/.bin/esbuild monitor.js --bundle --format=iife --minify --target=es2020 --outfile=web/monitor.js
```
