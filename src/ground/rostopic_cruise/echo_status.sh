#!/bin/bash
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
rostopic echo /carla/ego_vehicle/vehicle_status
