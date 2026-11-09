#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
CONTAINER=rviz_ego_map_demo
IMAGE=ego-swarm-map-demo:noetic
if [ "${1:-}" = --stop ]; then
  if docker container inspect "$CONTAINER" >/dev/null 2>&1; then docker stop "$CONTAINER"; fi
  exit 0
fi
if [ "${1:-}" = --goal ]; then
  [ "$#" -eq 3 ] || { printf '用法：ego-map-demo --goal X Y\n' >&2; exit 2; }
  exec docker exec "$CONTAINER" /opt/ego-display/scripts/entrypoint.sh python3 /opt/ego-map-demo/scripts/goal.py "$2" "$3"
fi
case "${1:-}" in
  -h|--help) printf '%s\n' 'ego-map-demo             打开今天地图的离线 EGO 点击规划窗口' 'ego-map-demo --goal X Y  为离线规划设置目标点' 'ego-map-demo --stop      关闭离线演示' '起点：建图原点；起终点高度：地面以上约1.5米。';exit 0 ;;
  '') ;;
  *) printf '未知选项：%s\n' "$1" >&2;exit 2 ;;
esac
source /home/zjh/mid360s_ws/docker/ros-noetic-rviz/scripts/common.sh
CONFIG_DIR="$ROOT"
IMAGE_TAG="$IMAGE"
check_docker;check_image;check_nvidia_runtime;check_gpu_host;check_display_env
mkdir -p "$ROOT/home/.rviz" "$ROOT/runtime" "$ROOT/results"
make_xauth_file || true
ensure_xhost_grant
map="/home/zjh/.local/share/ego-swarm-rviz/maps/current/map_world.pcd"
[ -f "$map" ] || die "今天地图的本机缓存不存在：$map"
[ -f "$ROOT/home/.rviz/ego_map_demo.rviz" ] || cp "$ROOT/config/ego_map_demo.rviz" "$ROOT/home/.rviz/ego_map_demo.rviz"
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || true)" = true ]; then
  log "离线规划窗口已打开。";exit 0
fi
if docker container inspect "$CONTAINER" >/dev/null 2>&1; then docker rm "$CONTAINER" >/dev/null; fi
docker run -d --name "$CONTAINER" --network none --ipc=host --gpus all \
  --user "$(id -u):$(id -g)" -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/.docker.xauth -e QT_X11_NO_MITSHM=1 \
  -e DISABLE_ROS1_EOL_WARNINGS=1 -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v "$ROOT/runtime/.docker.xauth:/tmp/.docker.xauth:ro" \
  -v "$ROOT/home:/home/ros" -v "$ROOT/results:/results" -v "$(realpath "$map"):/maps/selected.pcd:ro" \
  -v "$ROOT/scripts:/opt/ego-map-demo/scripts:ro" -v "$ROOT/config:/opt/ego-map-demo/config:ro" \
  "$IMAGE" bash /opt/ego-map-demo/scripts/start.sh
printf '%s\n' '离线窗口已打开。使用 2D Nav Goal 点击并短拖目标；目标高度自动取约1.5米离地。'
