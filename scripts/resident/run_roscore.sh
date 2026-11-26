#!/usr/bin/env bash
# 常驻 roscore：systemd mid360s-roscore.service 的 ExecStart（安装见同目录 install.sh）。
# 开机就起、不随 fusion / 手持测试 / 建图进退：MAVROS 常驻在它上面，fusion 没跑时也能读到飞控状态。
# master 走板内固定地址（scripts/ros_net.sh）：先等 ros0 上的 ${ORANGE_ROS_MASTER_IP} 建立（NetworkManager 开机建），
# 最多 60 s；等不到就失败退出，由 systemd 隔几秒重试。
DIR="$(cd "$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")" && pwd)"

for attempt in $(seq 60); do
  source "$DIR/../ros_net.sh" 2>/dev/null && break
  if [ "$attempt" = 60 ]; then
    echo "板内 ROS 地址 60 s 内未就绪（检查 ros0 或 ORANGE_ROS_MASTER_IP）" >&2
    exit 1
  fi
  sleep 1
done

source /opt/ros/noetic/setup.bash
exec roscore
