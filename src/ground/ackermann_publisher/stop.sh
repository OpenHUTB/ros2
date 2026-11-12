#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
# 停止：速度 0
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{speed: 0.0, steering_angle: 0.0}" -r 10
