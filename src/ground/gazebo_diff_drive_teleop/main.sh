#!/usr/bin/env bash
# Gazebo 差速小车：编译并一键启动。
# 用法:
#   bash main.sh          # 自动巡航演示
#   bash main.sh demo     # 自动巡航演示
#   bash main.sh manual   # 仅启动仿真，另开终端运行键盘节点
#   bash main.sh teach    # 启动仿真并进入示教/回放键盘节点
#   bash main.sh battery  # 启动仿真并进入电量管理/自动返航节点
#   bash main.sh precision # 启动仿真并进入里程计闭环精确运动节点
#   bash main.sh cruise   # 启动仿真并进入定速巡航/航向保持节点
set -e

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
colcon build --base-paths src --packages-select gazebo_diff_drive_teleop
source install/setup.bash

MODE="${1:-demo}"
if [ "$#" -gt 0 ]; then
    shift
fi

if [ "$MODE" = "teach" ] || [ "$MODE" = "battery" ] \
        || [ "$MODE" = "precision" ] || [ "$MODE" = "cruise" ]; then
    if [ "$MODE" = "teach" ]; then
        NODE_NAME=teach_replay
        LOG_NAME=teach_replay
        MODE_BANNER="键盘示教/回放"
    elif [ "$MODE" = "battery" ]; then
        NODE_NAME=battery_teleop
        LOG_NAME=battery
        MODE_BANNER="电量管理/自动返航"
    elif [ "$MODE" = "precision" ]; then
        NODE_NAME=precision_moves
        LOG_NAME=precision
        MODE_BANNER="里程计闭环精确运动"
    else
        NODE_NAME=cruise_control
        LOG_NAME=cruise
        MODE_BANNER="定速巡航/航向保持"
    fi
    mkdir -p "$WS_ROOT/log"
    LAUNCH_LOG="$WS_ROOT/log/gazebo_diff_drive_${LOG_NAME}_launch.log"

    echo "==> 后台启动 Gazebo 和控制器"
    setsid ros2 launch gazebo_diff_drive_teleop \
        gazebo_diff_drive_teleop.launch.py demo:=false "$@" \
        >"$LAUNCH_LOG" 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -INT -- -"$LAUNCH_PID" 2>/dev/null || true
            for _ in $(seq 1 20); do
                kill -0 "$LAUNCH_PID" 2>/dev/null || break
                sleep 0.1
            done
            kill -TERM -- -"$LAUNCH_PID" 2>/dev/null || true
            wait "$LAUNCH_PID" 2>/dev/null || true
        fi
    }
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM

    echo "==> 等待控制器和 /odom 就绪"
    READY=0
    for _ in $(seq 1 30); do
        if timeout 2s ros2 topic echo /odom --once >/dev/null 2>&1; then
            READY=1
            break
        fi
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            echo "[错误] 仿真进程已退出，日志：" >&2
            tail -n 30 "$LAUNCH_LOG" >&2
            exit 1
        fi
        sleep 1
    done
    if [ "$READY" -ne 1 ]; then
        echo "[错误] 等待 /odom 超时，仿真日志：" >&2
        echo "  $LAUNCH_LOG" >&2
        tail -n 30 "$LAUNCH_LOG" >&2
        exit 1
    fi

    echo "==> 仿真就绪，进入${MODE_BANNER}（按 q 退出）"
    echo "    仿真日志: $LAUNCH_LOG"
    set +e
    ros2 run gazebo_diff_drive_teleop "$NODE_NAME"
    STATUS=$?
    set -e
    exit "$STATUS"
fi

if [ "$MODE" = "manual" ]; then
    echo "==> 启动 Gazebo 仿真（手动模式，另开终端运行: ros2 run gazebo_diff_drive_teleop teleop_keyboard）"
    ros2 launch gazebo_diff_drive_teleop \
        gazebo_diff_drive_teleop.launch.py demo:=false "$@"
else
    echo "==> 启动 Gazebo 仿真 + 自动巡航演示"
    ros2 launch gazebo_diff_drive_teleop \
        gazebo_diff_drive_teleop.launch.py "$@"
fi
