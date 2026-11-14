#!/bin/bash
# task3_slam_nav 一键启动脚本（Linux / Ubuntu 虚拟机）
# 用法: ./main.sh  [goal_x goal_y]
set -e
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$SCRIPT_DIR"

echo "=== task3_slam_nav 启动 ==="
python3 main.py "$@"
