#!/bin/bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
rostopic type /carla/ego_vehicle/vehicle_control_cmd
rostopic hz /carla/ego_vehicle/vehicle_control_cmd
