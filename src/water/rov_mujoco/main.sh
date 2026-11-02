#!/bin/bash
# 水下机器人仿真一键运行脚本 (课程规范主入口)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 确保 WSLg 图形视窗环境变数
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi
if [ -z "$WAYLAND_DISPLAY" ]; then
    export WAYLAND_DISPLAY=wayland-0
fi
if [ -z "$XDG_RUNTIME_DIR" ]; then
    export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
fi

# Source ROS2 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source 工作空间环境 (若已编译)
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# 切换到包根目录并确保当前源码树优先加载
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

echo "=================================================="
echo "  水下机器人操作系统及应用 - 模块一键运行脚本"
echo "=================================================="

# 运行主入口
python3 main.py "$@"
