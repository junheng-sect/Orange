#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
SHARED="${EGO_RVIZ_SHARED:-/home/zjh/mid360s_ws/docker/ros-noetic-rviz}"
CONTAINER=rviz_ego_swarm
EGO_IMAGE=ego-swarm-rviz:noetic
mode=show
odom=/drone_nav/odom
master_override=""
ip_override=""
map_file="$ROOT/maps/current/map_world.pcd"
map_required=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --stop) mode=stop; shift ;;
    --check) mode=check; shift ;;
    --odom) odom="$2"; shift 2 ;;
    --master) master_override="$2"; shift 2 ;;
    --ros-ip) ip_override="$2"; shift 2 ;;
    --map) map_file="$2"; map_required=1; shift 2 ;;
    --no-map) map_file=""; shift ;;
    -h|--help)
      printf '%s\n' 'ego-rviz                    打开独立 EGO-Swarm 窗口（有默认地图时自动显示）' 'ego-rviz --check            检查真实数据发布者' 'ego-rviz --stop             关闭此窗口' 'ego-rviz --odom TOPIC       切换模型里程计输入（默认 /drone_nav/odom）' 'ego-rviz --map PCD_OR_DIR   显示指定本机地图文件或目录' 'ego-rviz --no-map           使用原实时视图，不加载保存地图'
      exit 0 ;;
    *) printf '未知参数：%s\n' "$1" >&2; exit 2 ;;
  esac
done
if [ "$mode" = stop ]; then
  if docker container inspect "$CONTAINER" >/dev/null 2>&1; then docker stop "$CONTAINER"; fi
  exit 0
fi
[ -r "$SHARED/scripts/common.sh" ] || { printf '缺少现有 RViz 网络辅助脚本：%s\n' "$SHARED" >&2; exit 1; }
# Reuse network/display checks only; all layouts, HOME, logs and the container
# are separate from the user's existing FAST-LIO / relocalization view.
source "$SHARED/scripts/common.sh"
CONFIG_DIR="$ROOT"
IMAGE_TAG="$EGO_IMAGE"
CONTAINER_NAME="$CONTAINER"
mkdir -p "$ROOT/home/.rviz" "$ROOT/runtime" "$ROOT/logs"
master="${master_override:-$(resolve_ros_master)}"
host="$(printf '%s' "$master" | sed -E 's#^[a-z]+://([^:/]+).*#\1#')"
if [ "$host" = "$BOARD_CORE_IP" ]; then ensure_core_route "$DEFAULT_BOARD_IP"; fi
ros_ip="${ip_override:-$(detect_ros_ip "$host")}"
check_docker
check_image
check_ros_master "$master" >/dev/null || die "ROS Master 不可达：$master"
if [ "$mode" = check ]; then
  exec docker run --rm --network host --user "$(id -u):$(id -g)" -e ROS_MASTER_URI="$master" -e ROS_IP="$ros_ip" "$EGO_IMAGE" python3 /opt/ego-display/scripts/check_inputs.py
fi
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || true)" = true ]; then
  log "EGO-Swarm 窗口已运行。重开前可用 ego-rviz --stop。"
  exit 0
fi
check_nvidia_runtime
check_gpu_host
check_display_env
make_xauth_file || true
ensure_xhost_grant
layout=ego_swarm_real.rviz
container_map=""
map_mount=()
if [ -n "$map_file" ]; then
  if [ -d "$map_file" ]; then map_file="$map_file/map_world.pcd"; fi
  if [ -f "$map_file" ]; then
    map_file="$(realpath "$map_file")"
    layout=ego_swarm_map.rviz
    container_map=/maps/selected.pcd
    map_mount=(-v "$map_file:$container_map:ro")
    log "保存地图=$map_file；固定坐标系=drone_map"
  elif [ "$map_required" -eq 1 ]; then die "地图文件不存在：$map_file"; fi
fi
[ -f "$ROOT/home/.rviz/$layout" ] || cp "$ROOT/rviz/$layout" "$ROOT/home/.rviz/$layout"
if docker container inspect "$CONTAINER" >/dev/null 2>&1; then docker rm "$CONTAINER" >/dev/null; fi
log "ROS_MASTER_URI=$master ROS_IP=$ros_ip，模型输入=$odom"
docker run -d --name "$CONTAINER" --network host --ipc=host --gpus all \
  --hostname "$(hostname)" --user "$(id -u):$(id -g)" \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e DISABLE_ROS1_EOL_WARNINGS=1 \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  -e ROS_MASTER_URI="$master" -e ROS_IP="$ros_ip" -e XAUTHORITY=/tmp/.docker.xauth \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$ROOT/runtime/.docker.xauth:/tmp/.docker.xauth:ro" \
  -v "$ROOT/home:/home/ros" -v "$ROOT/logs:/logs" \
  "${map_mount[@]}" \
  "$EGO_IMAGE" roslaunch /opt/ego-display/launch/real_display.launch "odom:=$odom" "config:=/home/ros/.rviz/$layout" "map_file:=$container_map"
printf '%s\n' 'EGO-Swarm 独立窗口已启动；停止：ego-rviz --stop'
