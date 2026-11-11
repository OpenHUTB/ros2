#!/usr/bin/env bash
# 统一入口：直接透传参数，例如 ./main.sh --probe --host 192.168.148.1
set -e
cd "$(dirname "$0")"
exec python3 main.py "$@"