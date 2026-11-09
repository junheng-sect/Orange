#!/usr/bin/env bash
set -euo pipefail
core="";planner="";view="";anchor=""
cleanup() {
  for p in "$view" "$planner" "$anchor" "$core"; do [ -z "$p" ] || kill "$p" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT INT TERM
roscore >/results/roscore.log 2>&1 & core=$!
python3 - <<'PY'
import time,xmlrpc.client
for _ in range(100):
 try:
  xmlrpc.client.ServerProxy('http://127.0.0.1:11311').getPid('/demo_start')
  break
 except OSError:time.sleep(.1)
else:raise RuntimeError('Local master failed to start')
PY
rosparam load /opt/ego-map-demo/config/planner.yaml /ego_demo
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world offline_map_origin >/results/anchor.log 2>&1 & anchor=$!
/opt/ego-map-demo/bin/ego_map_planner >/results/planner.log 2>&1 & planner=$!
rviz -d /home/ros/.rviz/ego_map_demo.rviz __name:=offline_map_rviz >/results/rviz.log 2>&1 & view=$!
wait -n "$core" "$planner" "$view"
