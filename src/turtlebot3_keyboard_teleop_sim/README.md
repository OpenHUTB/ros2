# turtlebot3_keyboard_teleop_sim

Keyboard teleoperation of TurtleBot3 in Gazebo (course task 1).

## Run environment
- Ubuntu 20.04, ROS Noetic, Gazebo 11 (course VM), Python 3.8

## Run steps
1. `source /opt/ros/noetic/setup.bash`
2. `python3 src/turtlebot3_keyboard_teleop_sim/scripts/main.py`
3. Drive with `方向键` in the xterm window opened by the launch file;
   stop this monitor with `Ctrl+C`.

World-only start:
`roslaunch src/turtlebot3_keyboard_teleop_sim/launch/keyboard_teleop_sim.launch`

## Parameters
- `TURTLEBOT3_MODEL`: robot model, default `burger`;
- `--gl software|hardware`: rendering backend, default `software` for VMs;
  `LIBGL_ALWAYS_SOFTWARE` in the shell overrides it;
- launch timeout: main.py waits up to 120 s for Gazebo services.

## Interfaces
- publishes `/cmd_vel` (geometry_msgs/Twist) from the teleop node;
- available on the robot: `/scan` (sensor_msgs/LaserScan), `/odom`;
- gazebo services used for readiness check: `/gazebo/get_world_properties`.

## Flow
main.py -> roslaunch module launch -> turtlebot3_world + teleop node in xterm
-> main.py polls roslaunch liveness and gazebo service -> monitors until exit.

## Test
`python3 -m unittest tests/test_keyboard_teleop_sim_local.py -v`

## Demo
![teleop demo](../../docs/img/ground/turtlebot3_keyboard_teleop_sim/teleop_demo.gif)
