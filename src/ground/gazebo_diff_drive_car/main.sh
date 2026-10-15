#!/bin/bash
# =============================================================================
# main.sh —— OpenHUTB 无人车（Gazebo）启动脚本
# =============================================================================
#
# 本脚本为任务书所要求的「模块统一入口」的 shell 形式。
# 仓库内其他模块（如 carla_keyboard_control）同样提供 main.sh / main.bat，
# 保持约定一致。
#
# 用法：
#     ./main.sh                       # 默认键盘遥控模式
#     ./main.sh laser                 # 激光雷达避障
#     ./main.sh camera                # 深度相机避障
#     ./main.sh teleop                # 键盘遥控（显式指定）
#     ./main.sh idle                  # 仅加载仿真环境
#     ./main.sh laser nogui           # 激光避障 + 无图形界面
#
# 参数：
#     $1  运行模式：teleop | laser | camera | idle   （默认 teleop）
#     $2  可选：nogui  —— 关闭 Gazebo 图形界面（适用于服务器/CI）
#
# 环境要求：
#     Ubuntu 20.04 + ROS Noetic + Gazebo 11
#
# =============================================================================

set -e

# ---------------------------- 参数解析 --------------------------------------
MODE="${1:-teleop}"
GUI="true"

if [ "$2" = "nogui" ] || [ "$2" = "headless" ]; then
    GUI="false"
fi

# ---------------------------- 环境准备 --------------------------------------
# 载入 ROS 环境（若尚未载入）
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/noetic/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/noetic/setup.bash
    else
        echo "[错误] 未找到 /opt/ros/noetic/setup.bash"
        echo "       请确认已安装 ROS Noetic。"
        exit 1
    fi
fi

# 载入本工作区（若已编译）
WORKSPACE_SETUP="$(cd "$(dirname "$0")/../../.." && pwd)/devel/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$WORKSPACE_SETUP"
fi

# Gazebo 在虚拟机 / 软件渲染环境下需要关闭 VGPU，否则可能黑屏或崩溃
export SVGA_VGPU10=0

# ---------------------------- 启动 ------------------------------------------
echo "=============================================================="
echo "  OpenHUTB 无人车 —— Gazebo 差速驱动仿真"
echo "=============================================================="
echo "  运行模式 : $MODE"
echo "  图形界面 : $GUI"
echo "  ROS 发行版: ${ROS_DISTRO:-未载入}"
echo "=============================================================="
echo ""

case "$MODE" in
    teleop|laser|camera|idle)
        roslaunch gazebo_diff_drive_car main.launch mode:="$MODE" gui:="$GUI"
        ;;
    *)
        echo "[错误] 未知模式：$MODE"
        echo "       支持的模式：teleop | laser | camera | idle"
        exit 1
        ;;
esac
