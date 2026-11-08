#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

# 虚拟机 / 无独立显卡环境：强制 Mesa 软件渲染，避免 MuJoCo viewer 黑屏
export LIBGL_ALWAYS_SOFTWARE=1

# 透传参数：bash run.sh --headless 可无窗口跑完并出图
python3 scripts/main.py "$@"
