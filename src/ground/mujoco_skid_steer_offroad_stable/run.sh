#!/bin/bash
cd "$(dirname "$0")"
# 软件渲染，虚拟机必备，避免黑屏
export MJ_CORE_GLFW=software
# noetic python3
python3 scripts/main.py

