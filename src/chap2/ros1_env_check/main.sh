#!/bin/bash
# 一键包装：在任意目录下都能调用同目录的 check_env.py
#
# 用法：
#   ./main.sh                 # 只检查环境
#   ./main.sh --scan .        # 同时扫描当前目录的文件卫生
#   ./main.sh --scan . --strict
#
# 说明：脚本用 BASH_SOURCE 定位自身目录，因此不要求 cd 到本目录再执行。

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3}"

if ! command -v "$PY" >/dev/null 2>&1; then
    echo "[FAIL] 找不到 $PY，请先安装 Python 3（sudo apt-get install -y python3）" >&2
    exit 1
fi

exec "$PY" "$HERE/check_env.py" "$@"
