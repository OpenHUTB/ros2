#!/usr/bin/env bash
# 一键入口（仓库约定：入口为 main.*）。
# 用法：./main.sh --host 192.168.148.1
set -e
cd "$(dirname "$0")"
python3 main.py "$@"
