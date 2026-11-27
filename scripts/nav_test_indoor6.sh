#!/usr/bin/env bash
# ==============================================================================
# 室内 test6：地面人工解锁后等 3 秒，原地爬升 0.50 m，执行 test5 扫码并等高返航慢降。
# 返航 XY 取实际起飞点，地图高度取右下观察点。飞行中切出 OFFBOARD 即结束本轮。
# --preview 用开始预览位置作为起飞 XY，仅执行空中航线，手持或模拟跟随，无飞控输出。
# --scan-save-frames off|failed|all  扫码 PNG 保存模式，默认 off
# --capture-waypoints               保存观察点全分辨率 PNG，默认关闭
# ==============================================================================
TEST_NAME=test_indoor6
source "$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")/nav_test/indoor_label_scan.sh"
nav_test_main "$@"
