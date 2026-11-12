#!/bin/bash
# R2D2 航点巡航仿真启动脚本（Linux/macOS）
# 用法: ./main.sh

echo "启动 R2D2 小车航点巡航仿真..."
python3 "$(dirname "$0")/src/r2d2_waypoint_cruise/cruise.py"
