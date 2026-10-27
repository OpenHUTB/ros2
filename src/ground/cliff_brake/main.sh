#!/bin/bash
# 一键入口：安装 Python 依赖并启动三节点仿真（需先 source ROS 环境）
cd "$(dirname "$0")"
echo "正在安装依赖..."
pip3 install -r requirements.txt
echo "启动悬崖边缘紧急制动仿真..."
roslaunch cliff_brake main.launch
