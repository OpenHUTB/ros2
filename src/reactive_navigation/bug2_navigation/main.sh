#!/usr/bin/env bash
# bug2_navigation 的入口：等价于 python3 scripts/main.py（离线出图，不需要 ROS 2）。
# ROS 2 闭环请用 ros2 launch bug2_navigation bug2_navigation.launch.py。
set -euo pipefail
cd "$(dirname "$0")"

if [ -f scripts/main.py ]; then
  exec python3 scripts/main.py "$@"
fi
# colcon 安装后 main.sh 与 main.py 同处 lib/<包名>/
exec python3 main.py "$@"
