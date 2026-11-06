#!/usr/bin/env bash
# 地面载具（CARLA）一键启动脚本
# 用法：
#   ./run_ground_demo.sh rl     # 端到端强化学习：CARLA->bridge->训练/推理
#   ./run_ground_demo.sh slam   # SLAM+导航：CARLA->bridge->RTAB-Map->Nav2
# 适用：Ubuntu 20.04 + ROS2 Humble（Noetic 把 ros2 换 ros1、launch 换 noetic/ 下的 .launch）

set -e

MODE=${1:-rl}
CARLA_ROOT=${CARLA_ROOT:-"/opt/carla-simulator"}   # 按实际 CARLA 安装路径改
ROS_DISTRO=${ROS_DISTRO:-"humble"}

echo "[1/4] 启动 CARLA 模拟器..."
if [ -f "$CARLA_ROOT/CarlaUE4.sh" ]; then
  (cd "$CARLA_ROOT" && ./CarlaUE4.sh -RenderOffScreen &)
else
  echo "未找到 $CARLA_ROOT/CarlaUE4.sh，请先改脚本里的 CARLA_ROOT"
fi

echo "[2/4] 等待 8 秒让仿真器起来..."
sleep 8

echo "[3/4] source ROS 与工作区..."
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

echo "[4/4] 启动模块（模式=$MODE）"
if [ "$MODE" = "rl" ]; then
  echo "  -> 训练：ros2 launch carla_rl_driving carla_rl_train.launch.py"
  ros2 launch carla_rl_driving carla_rl_train.launch.py algo:=ppo arch:=cnn scenario:=town01
elif [ "$MODE" = "slam" ]; then
  echo "  -> 先建图，另开终端再跑：ros2 launch carla_slam_nav carla_nav2.launch.py"
  ros2 launch carla_slam_nav carla_slam.launch.py
else
  echo "未知模式 $MODE（用 rl 或 slam）"
  exit 1
fi
