#!/usr/bin/env bash
# 人形机器人模块入口：./main.sh --mode nav --scene corridor
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash 2>/dev/null || true
[ -f "$HOME/humanoid_ws/devel/setup.bash" ] && source "$HOME/humanoid_ws/devel/setup.bash"
exec python3 "$HERE/main.py" "$@"
