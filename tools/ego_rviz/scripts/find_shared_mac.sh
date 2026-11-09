#!/usr/bin/env bash
# ROOT is the EGO display directory, whether in the repository or installed separately.
find_rviz_shared() {
  local candidate
  if [ -n "${EGO_RVIZ_SHARED:-}" ]; then
    [ -r "$EGO_RVIZ_SHARED/scripts/common_mac.sh" ] || { printf 'EGO_RVIZ_SHARED 无效：%s\n' "$EGO_RVIZ_SHARED" >&2; return 1; }
    printf '%s' "$EGO_RVIZ_SHARED"; return
  fi
  for candidate in "${MID360S_WS:-}/docker/ros-noetic-rviz" \
    "$ROOT/../../../mid360s_ws/docker/ros-noetic-rviz" \
    "$HOME/Projects/mid360s_ws/docker/ros-noetic-rviz" \
    "$HOME/mid360s_ws/docker/ros-noetic-rviz"; do
    if [ -r "$candidate/scripts/common_mac.sh" ]; then
      (cd "$candidate" && pwd); return
    fi
  done
  printf '%s\n' '未找到 mid360s_ws 显示工具；设置 EGO_RVIZ_SHARED=/路径/docker/ros-noetic-rviz' >&2
  return 1
}
