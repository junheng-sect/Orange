#!/usr/bin/env bash
# 安装 / 更新 / 卸载常驻 roscore 和 MAVROS 两个 systemd 服务（板端执行，要 sudo）。
#   ./install.sh              根据当前工作空间生成单元并安装 到 /etc/systemd/system，开机自启并立即（重新）启动
#   ./install.sh --uninstall  停掉、取消自启并删除这两个服务（之后 fusion.sh 等入口会报「常驻 roscore 没起来」）
#   ./install.sh --render DIR  仅生成两份单元供离线校验
# 移动检出或变更单元模板后需重装；run_roscore.sh / run_mavros.sh / mavros.launch 改了只要
# sudo systemctl restart mid360s-roscore（MAVROS 跟着重启）。
set -euo pipefail
DIR="$(cd "$(dirname -- "$(realpath -e -- "${BASH_SOURCE[0]}")")" && pwd)"
UNITS=(mid360s-roscore.service mid360s-mavros.service)

render_units() {
  python3 - "$DIR" "$1" "${UNITS[@]}" <<'PYTHON'
from pathlib import Path
import sys

templates = Path(sys.argv[1])
out = Path(sys.argv[2])
workspace = str(templates.parents[1])
if any(ord(c) < 32 for c in workspace):
    sys.exit("工作空间路径不能包含控制字符")
out.mkdir(parents=True, exist_ok=True)
for unit in sys.argv[3:]:
    template = (templates / (unit + ".in")).read_text()
    (out / unit).write_text(template.replace("@WORKSPACE@", workspace.replace("%", "%%") + "/."))
PYTHON
}

if [[ $# == 2 && $1 == --render ]]; then
  render_units "$2"
  exit 0
fi
if [[ $# -gt 0 && ( $# != 1 || $1 != --uninstall ) ]]; then
  echo "用法：$0 [--uninstall | --render DIR]" >&2
  exit 2
fi
if [ "${1:-}" = "--uninstall" ]; then
  sudo systemctl disable --now "${UNITS[@]}" || true
  for u in "${UNITS[@]}"; do sudo rm -f "/etc/systemd/system/$u"; done
  sudo systemctl daemon-reload
  echo "已卸载：${UNITS[*]}"
  exit 0
fi

rendered="$(mktemp -d)"
trap 'rm -rf -- "$rendered"' EXIT
render_units "$rendered"
for u in "${UNITS[@]}"; do
  sudo install -m 644 "$rendered/$u" "/etc/systemd/system/$u"
done
sudo systemctl daemon-reload
sudo systemctl enable "${UNITS[@]}"
# restart 而不是 start：更新时让新单元文件马上生效（MAVROS 随 roscore 一起重启）
sudo systemctl restart mid360s-roscore.service mid360s-mavros.service
systemctl --no-pager --lines=0 status "${UNITS[@]}" || true
