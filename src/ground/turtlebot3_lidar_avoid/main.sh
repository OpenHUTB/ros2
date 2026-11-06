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
#   - 已安装 ROS 2（推荐 ROS 2 Humble，脚本支持自动检测环境）
#   - TurtleBot3 仿真环境已启动（Gazebo 中已 spawn 机器人）
#   - 本包已编译：colcon build --packages-select turtlebot3_lidar_avoid
#
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---- 载入 ROS 2 环境 ----
if command -v ros2 >/dev/null 2>&1 && [ -n "$ROS_DISTRO" ]; then
    echo "[信息] 使用当前已载入的 ROS 2 环境: $ROS_DISTRO"
elif [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/iron/setup.bash
elif [ -f /opt/ros/rolling/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/rolling/setup.bash
else
    echo "[错误] 未找到可用的 ROS 2 环境配置（已尝试 humble/jazzy/iron/rolling）。"
    echo "       请先手动执行 source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# ---- 寻找并载入工作空间 ----
WS_ROOT=""
CUR_DIR="$SCRIPT_DIR"
while [ -n "$CUR_DIR" ] && [ "$CUR_DIR" != "/" ] && [ "$CUR_DIR" != "." ]; do
    if [ -f "$CUR_DIR/install/setup.bash" ]; then
        WS_ROOT="$CUR_DIR"
        break
    fi
    PARENT_DIR="$(dirname "$CUR_DIR")"
    if [ "$PARENT_DIR" = "$CUR_DIR" ]; then
        break
    fi
    CUR_DIR="$PARENT_DIR"
done

# 若未找到已编译的 install/setup.bash，推断源码所在的工作空间根目录（以备 --build）
if [ -z "$WS_ROOT" ]; then
    CUR_DIR="$SCRIPT_DIR"
    while [ -n "$CUR_DIR" ] && [ "$CUR_DIR" != "/" ] && [ "$CUR_DIR" != "." ]; do
        if [ "$(basename "$CUR_DIR")" = "src" ]; then
            WS_ROOT="$(dirname "$CUR_DIR")"
            break
        fi
        PARENT_DIR="$(dirname "$CUR_DIR")"
        if [ "$PARENT_DIR" = "$CUR_DIR" ]; then
            break
        fi
        CUR_DIR="$PARENT_DIR"
    done
fi

if [ -n "$WS_ROOT" ] && [ -f "$WS_ROOT/install/setup.bash" ]; then
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
            if [ -z "$WS_ROOT" ]; then
                echo "[错误] 无法定位 colcon 工作空间根目录，请在工作空间根目录下手动执行 colcon build"
                exit 1
            fi
            ( cd "$WS_ROOT" && colcon build --packages-select turtlebot3_lidar_avoid )
            if [ -f "$WS_ROOT/install/setup.bash" ]; then
                # shellcheck disable=SC1091
                source "$WS_ROOT/install/setup.bash"
            fi
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
