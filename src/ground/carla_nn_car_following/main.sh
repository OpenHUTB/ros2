#!/usr/bin/env bash
# 一键入口（仓库约定：入口为 main.*）。
# 用法：./main.sh --mode scene --host 192.168.148.1 --scenario decelerate
set -e
cd "$(dirname "$0")"
exec python3.10 main.py "$@"
