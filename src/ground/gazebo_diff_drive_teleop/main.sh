#!/usr/bin/env bash
# Gazebo 差速小车键盘控制：统一入口，编译并一键启动整个模块。
# 用法:
#   bash main.sh          # 默认自动巡航演示（小车沿圆形轨迹行驶）
#   bash main.sh manual   # 仅启动仿真，另开终端用键盘控制
set -e

# 本脚本位于 <ws>/src/.../gazebo_diff_drive_teleop/main.sh，向上回溯找到工作空间根目录（含 src/）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$SCRIPT_DIR"
while [ "$WS_ROOT" != "/" ] && [ ! -d "$WS_ROOT/src" ]; do
    WS_ROOT="$(dirname "$WS_ROOT")"
done

if [ ! -d "$WS_ROOT/src" ]; then
    echo "[错误] 未找到 ROS2 工作空间（含 src 目录），请确认本包位于工作空间 src 下。" >&2
    exit 1
fi

cd "$WS_ROOT"
echo "==> 编译 gazebo_diff_drive_teleop 包"
colcon build --packages-select gazebo_diff_drive_teleop
source install/setup.bash

MODE="${1:-demo}"
if [ "$MODE" = "manual" ]; then
    echo "==> 启动 Gazebo 仿真（手动模式，另开终端运行: ros2 run gazebo_diff_drive_teleop teleop_keyboard）"
    ros2 launch gazebo_diff_drive_teleop gazebo_diff_drive_teleop.launch.py demo:=false
else
    echo "==> 启动 Gazebo 仿真 + 自动巡航演示"
    ros2 launch gazebo_diff_drive_teleop gazebo_diff_drive_teleop.launch.py
fi
