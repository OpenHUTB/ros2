#!/usr/bin/env bash
# 终端 3：以 20 Hz 发布行驶控制命令（油门 0.5、直行、无刹车）
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
rostopic pub -r 20 /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 0.5, steer: 0.0, brake: 0.0, hand_brake: false, reverse: false, gear: 1, manual_gear_shift: false}"
