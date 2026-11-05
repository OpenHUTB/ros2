#!/bin/bash
# start.sh —— 一键启动（无需 catkin 编译，直接跑源码）
# 用法: bash start.sh
# 依赖: Ubuntu 20.04 + ROS Noetic + pip3 install mujoco
set -e

# 1) 先启动 roscore（若已运行会自动跳过）
if ! rostopic list >/dev/null 2>&1; then
    echo "[start.sh] starting roscore ..."
    gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash; roscore; exec bash"
    sleep 4
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 2) MuJoCo 仿真桥
echo "[start.sh] starting mujoco bridge ..."
gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash; cd '$SCRIPT_DIR'; python3 scripts/main_mujoco_bridge.py; exec bash"
sleep 2

# 3) 抓放状态机
echo "[start.sh] starting pick-place FSM ..."
gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash; cd '$SCRIPT_DIR'; python3 scripts/main_pick_fsm.py; exec bash"

echo "[start.sh] done. 终端分别显示 bridge / FSM 日志，机械臂自动循环抓放。"
