# One DOF Robotic Arm Simulation

## Description
A one-degree-of-freedom robotic arm simulation package based on ROS Noetic and Gazebo. This package includes the URDF model, position controller configuration, launch files, and a demonstration control script.

## Environment
- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11

## Quick Start

### 1. Build
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Launch Gazebo Simulation
```bash
roslaunch one_dof_arm arm_gazebo.launch
```

### 3. Run Control Demo
Open a new terminal:
```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun one_dof_arm main.py
```

## File Structure
```
one_dof_arm/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── arm_controller.yaml      # Controller PID parameters
├── launch/
│   └── arm_gazebo.launch        # Main launch file
├── urdf/
│   └── one_dof_arm.urdf         # Robot URDF model
└── scripts/
    └── main.py                  # Control demo script
```

## Joint Control
The arm has one revolute joint `arm_joint` rotating around Y-axis:
- Joint limit: [-1.57, 1.57] radians (±90°)
- Position controller: `position_controllers/JointPositionController`
- Command topic: `/arm_joint_position_controller/command` (Float64, unit: radian)

## Disclaimer
This module was developed with the assistance of a large language model. The author takes full responsibility for the submitted content.
