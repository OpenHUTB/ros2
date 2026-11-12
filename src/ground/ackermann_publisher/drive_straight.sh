#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
# 直行：速度 3 m/s，转角 0 rad
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{speed: 3.0, steering_angle: 0.0, steering_angle_velocity: 0.0, acceleration: 0.0, jerk: 0.0}" -r 10
