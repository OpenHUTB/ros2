#!/usr/bin/env bash
# main.sh —— 模块统一入口（等价于 python3 main.py "$@"）
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
exec python3 main.py "$@"
