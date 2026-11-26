# Ubuntu 上的 ROS Noetic 干净构建

在 Orange 根目录执行。基础镜像为固定摘要的 Ubuntu 20.04 / ROS Noetic Desktop Full，验收平台为 Linux amd64。镜像构建时下载依赖，工作空间编译和测试在断网容器内进行。

```bash
docker build -t orange/noetic-build:noetic docker/noetic-build

work="$(mktemp -d)"
git archive HEAD | tar -xf - -C "$work"
docker run --rm --pull=never --network none --cap-drop=ALL \
  --security-opt=no-new-privileges --user "$(id -u):$(id -g)" \
  --cpuset-cpus=0-7 --memory=13g -e HOME=/tmp -e GTSAM_JOBS=4 \
  --mount "type=bind,src=$work,dst=/ws" -w /ws \
  --entrypoint /bin/bash orange/noetic-build:noetic -ec \
  'scripts/build.sh -j4 -p3 --no-status && docker/noetic-build/test.sh'
```

`git archive HEAD` 只导出指定提交的受跟踪文件，新检出没有 `build/`、`devel/`、`.deps/`、个人 `.env` 或板端配置。CPU 编号和并发数可按主机资源调整。产物、catkin 日志和各包的 `build/<pkg>/test_results/` 留在 `$work`，下次干净验收重新创建目录。

## 依赖来源

- `scripts/build.sh` 从仓库 `third_party/` 编译 Livox-SDK2 和 GTSAM 4.0.3，安装到该检出的 `.deps/`，再调用 `catkin build`。
- Dockerfile 从官方源码构建只读 ZXing-C++ 3.0.0 和 AprilTag 3.4.5，分别校验 SHA-256。ZXing 的实现需要支持 `<format>` 的 C++20 标准库，镜像只为 ZXing 指定 GCC 13；catkin 包使用 Focal 默认编译器。Noetic 软件源的 AprilTag 3.2.0 在现有合成标签 ROI 位移回归上与板端版本的输出不同。
- 基础桌面镜像提供 Noetic、catkin、OpenCV、PCL 等；其余直接依赖在 Dockerfile 中安装，包括 catkin-tools、MAVROS 消息与节点、Ceres、GStreamer、FFmpeg 和测试用的 `pdfinfo`。FFmpeg 同时提供录像回归读取时间戳和帧索引所用的 `ffprobe`。Ubuntu Toolchain PPA 的密钥按指纹核验，仅用于容器环境。
- Focal 的 rosdep 键 `libzxing-dev` 会解析为旧包 `libzxingcore-dev`，Noetic 的 `apriltag` 二进制包也低于本项目要求。原生 Noetic 环境应先按 Dockerfile 安装这两个源码版本，再用 rosdep 安装其他依赖；对应跳过项为 `--skip-keys "libzxing-dev apriltag"`。

## 验收范围

默认发现 27 个包。catkin 在干净构建中另生成内部包 `catkin_tools_prebuild`。六个已有 `CATKIN_IGNORE` 的包保持按需启用。

测试入口运行 `catkin test` 和 `catkin_test_results build`，并补跑未注册的定位、导航与板端监控单元测试、RC 录包和单/双相机合成录像测试。需要真实地图、飞行 bag、摄像头、雷达、PX4 或 RK3588 专有插件的实机验收另行进行。电脑端监控使用 Python 3.11+，其路径与 SSH 模拟测试可在 Ubuntu 开发机运行：

```bash
python3 -B -m unittest discover -s tools/warehouse_monitor -p 'test_board*.py'
```
