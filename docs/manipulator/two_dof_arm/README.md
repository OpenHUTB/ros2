# Two DOF Robotic Arm Circle Trajectory

A 2-degree-of-freedom (2R) robotic arm simulated in Gazebo, drawing a circle trajectory using inverse kinematics.

## Overview

This package implements a planar 2-DOF robotic arm with:
- **Base link**: Blue fixed base (0.15m cube)
- **Link 1**: Red first arm (0.3m)
- **Link 2**: Green second arm (0.25m)
- **Joint 1 & 2**: Revolute joints rotating around Z axis

The control script computes inverse kinematics in real-time to make the end-effector trace a circle in the XY plane.

## Project Structure

```
two_dof_arm/
├── CMakeLists.txt
├── package.xml
├── config/arm_controller.yaml    # PID position controllers for both joints
├── launch/arm_gazebo.launch      # Gazebo simulation launch file
├── urdf/two_dof_arm.urdf         # Robot URDF model
└── scripts/main.py               # Inverse kinematics + circle trajectory
```

## Dependencies

- ROS Noetic
- Gazebo
- ros_control (position_controllers, joint_state_controller)

## Build and Run

```bash
# Build
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Terminal 1: Launch Gazebo simulation
roslaunch two_dof_arm arm_gazebo.launch

# Terminal 2: Run circle drawing controller
rosrun two_dof_arm main.py
```

## How It Works

1. The URDF defines two revolute joints (joint1, joint2) controlled by position controllers.
2. `main.py` publishes target joint angles at 50 Hz.
3. Given a desired end-effector position (x, y), inverse kinematics computes:
   - theta2 = acos((x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2))
   - theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
4. The circle is parameterized as x = cx + r*cos(omega*t), y = cy + r*sin(omega*t).

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| cx, cy    | 0.38, 0.20 | Circle center (m) |
| r         | 0.08    | Circle radius (m) |
| omega     | 0.5     | Angular velocity (rad/s) |
| L1, L2    | 0.3, 0.25 | Link lengths (m) |

## Notes

- Add `<gazebo reference="...">` blocks in URDF to ensure colors render correctly in Gazebo.
- The elbow-up IK solution is used.

## Author

hjy
