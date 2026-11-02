#!/bin/bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
rostopic pub -r 20 /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 0.3, steer: 0.0, brake: 0.0, hand_brake: false, reverse: true, gear: -1, manual_gear_shift: false}"
