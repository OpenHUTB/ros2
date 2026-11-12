#!/usr/bin/env bash
# carla_lidar_obstacle_detection 入口：自动挑选能 import carla 的解释器后运行 main.py
set -e
cd "$(dirname "$0")"
for cand in python3.10 python3 python; do
    if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import carla" >/dev/null 2>&1; then
        echo "[info] 解释器: $cand ($("$cand" -c 'import sys; print(sys.version.split()[0])'))"
        exec "$cand" main.py "$@"
    fi
done
echo "[错误] 没有能 import carla 的解释器，请先安装 hutb-2.10.1 客户端 wheel"
exit 1
