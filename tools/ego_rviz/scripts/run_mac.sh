#!/usr/bin/env bash
set -euo pipefail
ROOT="$(python3 -c 'import os,sys; print(os.path.dirname(os.path.dirname(os.path.realpath(sys.argv[1]))))' "${BASH_SOURCE[0]}")"
source "$ROOT/scripts/find_shared_mac.sh"
SHARED="$(find_rviz_shared)"
source "$SHARED/scripts/common_mac.sh"
CONTAINER_NAME=rviz_ego_swarm
IMAGE_TAG="${EGO_RVIZ_IMAGE:-ego-swarm-rviz:noetic-arm64-web}"
image_build_script="$ROOT/scripts/build_image_mac.sh"
run_mode=rviz
odom=/drone_nav/odom
ros_master_uri="${RVIZ_ROS_MASTER:-}"
ros_ip_override="${RVIZ_ROS_IP:-}"
map_file="$ROOT/maps/current/map_world.pcd"
map_required=0
dry_run=0
detach=1
master_check=1
web_port="${EGO_RVIZ_WEB_PORT:-6081}"
open_browser=1
while [ "$#" -gt 0 ]; do
  case "$1" in
    --stop) run_mode=stop; shift ;;
    --check) run_mode=check; detach=0; shift ;;
    --odom) require_value "$@"; odom="$2"; shift 2 ;;
    --master|--ros-master) require_value "$@"; ros_master_uri="$2"; shift 2 ;;
    --ros-ip) require_value "$@"; ros_ip_override="$2"; shift 2 ;;
    --map) require_value "$@"; map_file="$2"; map_required=1; shift 2 ;;
    --no-map) map_file=''; shift ;;
    --web-port) require_value "$@"; web_port="$2"; shift 2 ;;
    --name) require_value "$@"; CONTAINER_NAME="$2"; shift 2 ;;
    --no-master-check) master_check=0; shift ;;
    --no-gpu) shift ;;
    --no-open) open_browser=0; shift ;;
    --dry-run) dry_run=1; shift ;;
    -h|--help)
      cat <<'EOF'
ego-rviz-mac [--map PCD_OR_DIR | --no-map] [--odom TOPIC]
  --check            检查真实话题接收与 TF；不启动显示
  --stop             停止此显示容器
  --master URI       ROS Master（也接受 --ros-master）
  --ros-ip IP        宿主机 ROS IP，默认自动识别
  --web-port PORT    Mac 浏览器端口，默认 6081（VNC 为 PORT+10000）
  --name NAME        独立容器名，默认 rviz_ego_swarm
  --no-master-check  跳过 Master 检查
  --no-gpu           兼容参数；Mac 固定使用软件渲染
  --no-open          启动 Mac 显示容器，不自动打开浏览器
  --dry-run          只打印命令，不操作 Docker、路由或个人文件
首次构建：tools/ego_rviz/scripts/build_image_mac.sh
Mac 使用 ARM64 Docker + 浏览器 RViz；Ubuntu 继续使用原 ego-rviz / run.sh。
EOF
      exit 0 ;;
    *) die "未知参数：$1" ;;
  esac
done
if [ "$run_mode" = stop ]; then stop_display; exit; fi
[ -n "$ros_master_uri" ] || ros_master_uri="$(resolve_ros_master)"
layout=ego_swarm_real.rviz
container_map=''
extra_mounts=(-v "$SHARED:/opt/rviz-ws:ro")
if [ -n "$map_file" ]; then
  [ ! -d "$map_file" ] || map_file="$map_file/map_world.pcd"
  if [ -f "$map_file" ]; then
    map_file="$(python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$map_file")"
    layout=ego_swarm_map.rviz
    container_map=/maps/selected.pcd
    extra_mounts+=(-v "$map_file:$container_map:ro")
  elif [ "$map_required" -eq 1 ]; then die "地图文件不存在：$map_file"; fi
fi
display_home="$ROOT/home"
display_logs="$ROOT/logs"
display_template="$ROOT/rviz/$layout"
personal_config="$ROOT/home/.rviz/$layout"
if [ "$run_mode" = check ]; then
  CONTAINER_NAME="${CONTAINER_NAME}_check_$$"
  fixed_frame=drone_odom
  [ -z "$container_map" ] || fixed_frame=drone_map
  display_command=(python3 /opt/rviz-ws/scripts/check_inputs.py --mode ego --odom "$odom" --fixed-frame "$fixed_frame")
else
  if [ "$dry_run" -eq 0 ]; then
    check_docker
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || true)" = true ]; then
      log "EGO-Swarm 窗口已运行；切换地图或配置前请先 --stop。"
      exit 0
    fi
  fi
  display_command=(roslaunch /opt/ego-display/launch/real_display.launch "odom:=$odom"
    "config:=/home/ros/.rviz/$layout" "map_file:=$container_map")
fi
run_display
