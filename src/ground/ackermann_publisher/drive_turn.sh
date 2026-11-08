#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
# 转向：速度 2 m/s，转角 0.3 rad
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{speed: 2.0, steering_angle: 0.3, steering_angle_velocity: 0, acceleration: 0, jerk: 0}" -r 10
