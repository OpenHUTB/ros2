#!/bin/bash
cd ~/carla-ros-bridge/catkin_ws
catkin_make
source devel/setup.bash
source /opt/ros/noetic/setup.bash
rosrun ackermann_publisher ackermann_pub.py
