#!/usr/bin/env bash
# airsim_multirotor_waypoint_nav 入口：自动挑选 Python 后运行 main.py
# 注意：本模块依赖 msgpack-rpc-python（只兼容 tornado<5），deps/ 里已放好 tornado 4.5.3，
#       main.py 会把它插到 sys.path 最前面，不影响系统 Jupyter 的 tornado 6。
set -e
cd "$(dirname "$0")"
for cand in python3.10 python3 python; do
    if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import airsim" >/dev/null 2>&1; then
        echo "[info] 解释器: $cand"
        exec "$cand" main.py "$@"
    fi
done
echo "[错误] 没有能 import airsim 的解释器。请先："
echo "       python -m pip install D:\\hutb\\PythonAPI\\carla\\dist\\hutb-2.10.1-cp310-cp310-win_amd64.whl"
echo "       python -m pip install msgpack-rpc-python"
exit 1
