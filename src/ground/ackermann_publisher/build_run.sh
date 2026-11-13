#!/bin/bash
cd ~/carla-ros-bridge/catkin_ws
catkin_make
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun ackermann_publisher ackermann_pub.py
