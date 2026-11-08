#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
