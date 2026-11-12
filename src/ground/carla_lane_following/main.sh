#!/usr/bin/env bash
# carla_lane_following 入口：自动挑选能 import carla 的解释器，然后透传所有参数给 main.py
set -e
cd "$(dirname "$0")"

pick_python() {
    for cand in python3.10 python3 python; do
        if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import carla" >/dev/null 2>&1; then
            echo "$cand"
            return 0
        fi
    done
    return 1
}

if ! PY=$(pick_python); then
    echo "[错误] 没有任何 Python 能 import carla。请先安装客户端 wheel："
    echo "       pip install ${HUTB_ROOT:-<OpenHUTB 安装目录>}/PythonAPI/carla/dist/hutb-2.10.1-cp310-cp310-win_amd64.whl"
    exit 1
fi

echo "[info] 解释器: $PY ($("$PY" -c 'import sys; print(sys.version.split()[0])'))"
exec "$PY" main.py "$@"
