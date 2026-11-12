#!/usr/bin/env bash
# 人形机器人模块入口：./main.sh --mode nav --scene corridor
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash 2>/dev/null || true
WS="${HUMANOID_WS:-$HOME/humanoid_ws}"
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS/devel/setup.bash"
else
  echo "[main] 未找到 $WS/devel/setup.bash；请先 catkin_make，或设置 HUMANOID_WS=/path/to/ws" >&2
fi
exec python3 "$HERE/main.py" "$@"
