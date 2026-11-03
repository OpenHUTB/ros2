#!/usr/bin/env bash
# 源码目录直接运行（不需要 colcon install）
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${HERE}:${HERE}/../uav_mujoco_sim:${PYTHONPATH}"
exec python3 "${HERE}/main.py" "$@"
