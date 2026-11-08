#!/usr/bin/env bash
# Linux / macOS 启动脚本：等价于在该目录下执行 python3 main.py "$@"
set -e
cd "$(dirname "$0")"
exec python3 main.py "$@"
