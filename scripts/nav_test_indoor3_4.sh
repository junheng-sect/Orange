#!/usr/bin/env bash
# ==============================================================================
# 室内 test3_4 · 手飞 OV13855 录像。飞机静置，等室内重定位确认和 OV13855 就绪，拨通道 6 同时录 bag 和 OV13855 无损视频。
# 全程遥控。通道 6 回拨后保留 1 秒尾段，降落上锁后 Ctrl+C 或从网页收尾。
# ==============================================================================
TEST_NAME=test_indoor3_4
NT_MANUAL_FLIGHT=1
SELF_DIR="$(cd "$(dirname -- "$(realpath -e -- "$0")")" && pwd)"
source "$SELF_DIR/nav_test/common.sh"
DEFAULT_MAP="$MAP_ROOT/20261114_101654_indoor_fly_raw_000001"
DEFAULT_RELOC_CONFIG="$WS/src/localization/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$WS/src/localization/car_bridge/config/map_frame_indoor_fly.yaml"
DEFAULT_NAV_CONFIG="$NT_DIR/indoor_fly_nav.yaml"
DEFAULT_EV_MODE=odom

test_desc() { echo "手飞 · 通道 6 同步录制定位 bag 和 OV13855 无损视频"; }
test_arg() { return 1; }

source "$NT_DIR/indoor_video.sh"

test_after_fusion() { video_after_fusion; }

run_test() {
  log "室内重定位已确认，手飞 OV13855 录像待命"
  local last="" state
  while :; do
    state="$(cat "$OUT/video_state.txt" 2>/dev/null)"
    child_pid "$PF_VIDEO" >/dev/null || state="录像器退出，请查看 video.log；定位继续运行"
    child_pid "$PF_RECORD" >/dev/null || state="录包器退出，请查看 record.log；定位继续运行"
    if [[ "$state" != "$last" ]]; then log "VIDEO34 $state"; last="$state"; fi
    child_pid "$PF_FUSION" >/dev/null && child_pid "$PF_RELOC" >/dev/null || return 1
    sleep 1
  done
}

nav_test_main "$@"
