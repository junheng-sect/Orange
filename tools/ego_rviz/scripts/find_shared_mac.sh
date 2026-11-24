#!/usr/bin/env bash
# ROOT is the directory of the selected EGO tool.
find_rviz_shared() {
  local shared="${EGO_RVIZ_SHARED:-$ROOT/../../docker/ros-noetic-rviz}"
  if [ ! -r "$shared/scripts/common_mac.sh" ]; then
    printf '未找到 RViz 共用脚本：%s（可设置 EGO_RVIZ_SHARED）\n' "$shared" >&2
    return 1
  fi
  (cd "$shared" && pwd -P)
}
