#!/bin/bash
# 等价入口：bash main.sh --headless
cd "$(dirname "$0")" || exit 1
exec python3 main.py "$@"
