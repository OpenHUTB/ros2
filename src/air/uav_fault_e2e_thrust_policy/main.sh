#!/usr/bin/env bash
# 源码树里直接跑（不需要 ROS）。装进工作空间后用 ros2 run / ros2 launch。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -eq 0 ]; then
    # 没给参数时跑自检，和 main.bat 保持一致
    exec python3 "${SCRIPT_DIR}/main.py" --selftest
fi

exec python3 "${SCRIPT_DIR}/main.py" "$@"
