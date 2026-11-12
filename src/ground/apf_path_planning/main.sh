#!/usr/bin/env bash
# 根目录入口：转发到 ROS2 包内的 main
cd "$(dirname "$0")"
python3 main.py "$@"