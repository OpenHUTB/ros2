#!/usr/bin/env bash
# mujoco_rov_depth_control 入口：自动挑选 Python 后运行 main.py
set -e
cd "$(dirname "$0")"
for cand in python3.10 python3 python; do
    if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import mujoco" >/dev/null 2>&1; then
        echo "[info] 解释器: $cand ($("$cand" -c 'import mujoco; print("mujoco", mujoco.__version__)'))"
        exec "$cand" main.py "$@"
    fi
done
echo "[错误] 没有能 import mujoco 的解释器，请先： python -m pip install mujoco"
exit 1
