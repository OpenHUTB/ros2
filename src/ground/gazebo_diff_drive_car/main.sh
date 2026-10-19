#!/bin/bash
# ============================================================================
# Gazebo 差速驱动小车 —— 一键运行脚本（课程 main.* 约定）
# ============================================================================
#
#   bash main.sh                      # 键盘遥控（默认）
#   bash main.sh laser                # 激光雷达避障
#   bash main.sh camera               # 深度相机避障
#   bash main.sh idle                 # 仅加载仿真环境
#   bash main.sh laser nogui          # 激光避障 + 无图形界面
#   bash main.sh --build [mode]       # 先 colcon 编译再运行
#
# 前置条件：
#   - ROS 2 Humble（/opt/ros/humble）
#   - Gazebo 11 与 gazebo_ros 插件
#   - 本包已编译：colcon build --packages-select gazebo_diff_drive_car
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

# ---- 载入工作空间 ----
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS_ROOT/install/setup.bash"
fi

# ---- 参数解析 ----
DO_BUILD=0
MODE="teleop"
GUI="true"

for arg in "$@"; do
    case "$arg" in
        --build) DO_BUILD=1 ;;
        nogui|headless) GUI="false" ;;
        teleop|laser|camera|idle) MODE="$arg" ;;
        --help|-h) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "[警告] 忽略未知参数：$arg" ;;
    esac
done

if [ "$DO_BUILD" = "1" ]; then
    echo "[1/2] colcon build ..."
    ( cd "$WS_ROOT" && colcon build --packages-select gazebo_diff_drive_car )
    # shellcheck disable=SC1091
    source "$WS_ROOT/install/setup.bash"
fi

echo "=================================================="
echo "  Gazebo 差速驱动小车"
echo "=================================================="
echo "  运行模式 : $MODE"
echo "  图形界面 : $GUI"
echo "=================================================="

exec ros2 launch gazebo_diff_drive_car main.launch.py mode:="$MODE" gui:="$GUI"
