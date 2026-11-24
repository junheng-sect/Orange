# shellcheck shell=bash
# source 本文件：板端 ROS 核心环境，不跟随 Wi-Fi。
# ROS_IP 用常驻 dummy 网卡 ros0 上的固定地址（NetworkManager 连接 ros-core，开机自动建立），
# ROS_MASTER_URI 走 127.0.0.1：Wi-Fi 没连、断开、换网、地址被收回，板端节点之间的连接都不受影响。
# 本机 RViz 经到 ${ORANGE_ROS_MASTER_IP}/32 的路由访问（下一跳是板端当前 Wi-Fi 地址），换网只影响远程连接。
# 地址不在时返回 1，ROS_IP / ROS_MASTER_URI 不动。ROS_NET_WIFI_IP 是 wlan0 当前地址（没连为空），只用于提示。
# 手动起节点前：source ~/mid360s_ws/scripts/ros_net.sh
ROS_CORE_IP="${ORANGE_ROS_MASTER_IP:-}"
ROS_CORE_IFACE="ros0"
ROS_NET_WIFI_IP="$(ip -4 -o addr show dev wlan0 scope global 2>/dev/null \
  | awk '{split($4, a, "/"); print a[1]; exit}')" || ROS_NET_WIFI_IP=""
_ros_net_addr="$(ip -4 -o addr show dev "$ROS_CORE_IFACE" 2>/dev/null)" || _ros_net_addr=""
if [ -z "$ROS_CORE_IP" ]; then
  _ros_env="$(dirname "${BASH_SOURCE[0]}")/../.env"
  if [ -r "$_ros_env" ]; then
    # 只读取地址，不执行 .env 中的其他设置。
    ROS_CORE_IP="$(awk -F= '$1 == "ORANGE_ROS_MASTER_IP" {gsub(/[[:space:]"\047]/, "", $2); print $2; exit}' "$_ros_env")"
  fi
  unset _ros_env
fi
case "$_ros_net_addr" in
  *" inet $ROS_CORE_IP/"*) ;;
  *)
    unset _ros_net_addr
    echo "[ros_net] 检查 ORANGE_ROS_MASTER_IP（环境变量或工作空间 .env）以及 $ROS_CORE_IFACE 的地址" >&2
    return 1 2>/dev/null || exit 1 ;;
esac
unset _ros_net_addr
export ROS_IP="$ROS_CORE_IP"
export ROS_MASTER_URI="http://127.0.0.1:11311"
unset ROS_HOSTNAME 2>/dev/null || true
