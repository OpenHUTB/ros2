#!/bin/bash
# 水下 ROV AprilTag 视觉定位与定点巡航仿真 - 一键启动脚本
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# 仓库根目录的 colcon 构建结果（src 上三级为仓库根目录）
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    source "$REPO_ROOT/install/setup.bash"
fi

cd "$SCRIPT_DIR"
echo "=================================================="
echo "  ROV AprilTag 视觉定位与定点巡航仿真"
echo "=================================================="
python3 main.py "$@"
