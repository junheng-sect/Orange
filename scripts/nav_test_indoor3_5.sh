#!/usr/bin/env bash
# ==============================================================================
# 室内 test3_5 四点减振测试。起飞前拨下通道 6，人工起飞后切 OFFBOARD。
# 右下保持，由飞手接管、落地上锁，再关通道 6。--damper-deg 20 可选。
# --preview 运行同一航线的无飞控输出预览。
# ==============================================================================
TEST_NAME=test_indoor3_5
SELF_DIR="$(cd "$(dirname -- "$(realpath -e -- "$0")")" && pwd)"
source "$SELF_DIR/nav_test/common.sh"
source "$NT_DIR/indoor_video.sh"
VIDEO_FULL=true
DAMPER_DEG=''
DEFAULT_MAP="$MAP_ROOT/20261114_101654_indoor_fly_raw_000001"
DEFAULT_RELOC_CONFIG="$WS/src/localization/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$WS/src/localization/car_bridge/config/map_frame_indoor_fly.yaml"
DEFAULT_NAV_CONFIG="$NT_DIR/indoor_fly_nav.yaml"
DEFAULT_EV_MODE=odom
DEFAULT_PREVIEW_RADIUS=0.4
DEFAULT_PREVIEW_HALF_HEIGHT=0.3
test_desc() { echo "四点减振对照，通道 6 全程录像，每点三张全图"; }
test_arg() {
  [[ "$1" == --damper-deg && "${2:-}" =~ ^(100|[1-9]?[0-9])$ ]] || return 1
  DAMPER_DEG="$2"; SHIFT=2
}
run_test() {
  nt_check_chain || return 1
  local flags=()
  if (( PREVIEW )); then
    local answer
    read -r -p "回车开始四点预览，q 退出：" answer </dev/tty || return 1
    [[ -z "$answer" ]] || return 0
    flags+=(--preview)
  fi
  log "四点减振 · ${DAMPER_DEG:-未标注}；通道 6 开始连录，右下由飞手接管"
  python3 -B "$NT_DIR/nav_indoor3_5.py" --out "$OUT" "${flags[@]}" 2>&1 | tee -a "$LOG"
}
# Persist the label even when no route is started during this CH6 segment.
test_after_fusion() {
  printf '%s\n' "${DAMPER_DEG:-未标注}" > "$OUT/damper_deg.txt" || return 1
  video_after_fusion || return 1
  python3 -B "$NT_DIR/nav_indoor3.py" --probe-camera --out "$OUT"
}
nav_test_main "$@"
