#!/bin/bash
cd ~/carla-ros-bridge/catkin_ws/src
catkin_create_pkg ackermann_publisher rospy ackermann_msgs
cd ackermann_publisher
mkdir scripts
cd scripts
touch ackermann_pub.py
chmod +x ackermann_pub.py
