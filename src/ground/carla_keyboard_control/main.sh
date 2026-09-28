#!/bin/bash
# CARLA 0.9.16 地面载具物理仿真与键盘运动控制 —— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                 # 独立模式（pygame 窗口 + 键盘控制）
#   bash main.sh --launch        # ROS 2 Humble launch 模式
#   bash main.sh --host <IP>     # 指定 CARLA 服务端地址（虚拟机填宿主机 IP）
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

# 加载 ROS 2 Humble 环境（若存在），供 --launch 模式使用
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

echo "=================================================="
echo "  CARLA 0.9.16 地面载具物理仿真与键盘运动控制"
echo "=================================================="

exec python3 main.py "$@"
