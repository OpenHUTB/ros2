#!/usr/bin/env bash
# 在 Ubuntu 虚拟机里的启动脚本（等价于 python3 main.py）
set -e
cd "$(dirname "$0")"
exec python3 main.py "$@"
