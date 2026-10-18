#!/bin/bash
# ============================================================================
# TurtleBot3 激光雷达自主避障 —— 一键运行脚本（课程 main.* 约定）
# ============================================================================
#
#   bash main.sh                     # 直接运行避障节点（默认 /scan）
#   bash main.sh /out/scan           # 指定雷达话题
#   bash main.sh --launch            # 通过 ROS 2 launch 启动
#   bash main.sh --build             # 先 colcon 编译再运行
#
# 前置条件：
#   - ROS 2 Humble（/opt/ros/humble）
#   - TurtleBot3 仿真环境已启动（Gazebo 中已 spawn 机器人）
#   - 本包已编译：colcon build --packages-select turtlebot3_lidar_avoid
#
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---- 载入 ROS 2 环境 ----
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "[错误] 未找到 /opt/ros/humble/setup.bash，请确认已安装 ROS 2 Humble"
    exit 1
fi

# ---- 载入工作空间（若本包处于 colcon 工作空间内）----
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS_ROOT/install/setup.bash"
fi

echo "=================================================="
echo "  TurtleBot3 激光雷达自主避障"
echo "=================================================="

# ---- 参数解析 ----
MODE="run"
SCAN_TOPIC="/scan"

for arg in "$@"; do
    case "$arg" in
        --launch) MODE="launch" ;;
        --build)
            echo "[1/2] colcon build ..."
            ( cd "$WS_ROOT" && colcon build --packages-select turtlebot3_lidar_avoid )
            # shellcheck disable=SC1091
            source "$WS_ROOT/install/setup.bash"
            ;;
        --help|-h)
            sed -n '2,20p' "$0"
            exit 0
            ;;
        *)
            SCAN_TOPIC="$arg"
            ;;
    esac
done

# ---- 启动 ----
if [ "$MODE" = "launch" ]; then
    echo "启动方式: ros2 launch（scan_topic=$SCAN_TOPIC）"
    exec ros2 launch turtlebot3_lidar_avoid main.launch.py "scan_topic:=$SCAN_TOPIC"
else
    echo "启动方式: ros2 run（scan_topic=$SCAN_TOPIC）"
    exec ros2 run turtlebot3_lidar_avoid avoid_node --ros-args -p "scan_topic:=$SCAN_TOPIC"
fi
