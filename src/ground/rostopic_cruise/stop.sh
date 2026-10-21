#!/usr/bin/env bash
# 终端 3：发布制动命令（油门 0、刹车 1），车辆逐渐减速停下
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
rostopic pub -r 20 /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 0.0, steer: 0.0, brake: 1.0, hand_brake: false, reverse: false, gear: 1, manual_gear_shift: false}"
