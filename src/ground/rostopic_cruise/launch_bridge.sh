#!/usr/bin/env bash
# 终端 2：启动 ROS 桥与主车（已包含加载环境步骤）
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
