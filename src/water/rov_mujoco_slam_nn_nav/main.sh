#!/usr/bin/env bash
# 模块入口：./main.sh --mode nav --scene trench
# 环境：Ubuntu 20.04 + ROS Noetic（含 ros-noetic-tf）+ Python 3.8 + MuJoCo 3.x
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash 2>/dev/null || true
for ws in "$HOME/rov_ws" "$HOME/water_ws"; do
  [ -f "$ws/devel/setup.bash" ] && source "$ws/devel/setup.bash"
done
exec python3 "$HERE/main.py" "$@"
