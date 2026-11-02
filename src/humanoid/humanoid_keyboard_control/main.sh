#!/bin/bash
# 人形机器人 MuJoCo 仿真与键盘运动控制一键运行脚本（课程规范主入口）

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 环境（若存在）
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source 工作空间环境（若已编译）
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# 切换到包根目录并确保当前源码树优先加载
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

echo "=================================================="
echo "  人形机器人操作系统及应用 - 模块一键运行脚本"
echo "=================================================="

# 运行主入口
python3 main.py "$@"
