#!/usr/bin/env bash
# 终端 3：查看车辆控制消息的字段定义
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
rosmsg show carla_msgs/CarlaEgoVehicleControl
