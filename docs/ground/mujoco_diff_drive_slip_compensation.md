# 基于编码器与惯性测量单元的 MuJoCo 差速小车轮滑感知与神经网络轨迹补偿

MuJoCo Differential-Drive Slip Perception and Neural Trajectory Compensation Using Wheel Encoders and an IMU

## 模块概述 / Overview

该模块使用 MuJoCo 3.12.0 构建四轮差速小车，在 ROS 2 Humble 中提供统一的速度控制、仿真时钟、真实里程计与坐标变换接口。系统以编码器和惯性测量单元数据为主要观测量，后续可在多摩擦条件下建立轮滑感知与轨迹补偿流程。

The module uses MuJoCo 3.12.0 to model a four-wheel differential-drive robot and exposes velocity control, simulation time, ground-truth odometry, and transform interfaces in ROS 2 Humble. Wheel encoder and inertial measurement unit data form the primary observations for subsequent slip perception and trajectory compensation across multiple friction conditions.

## 系统结构 / Architecture

```text
/cmd_vel
   |
   v
差速运动学 / Differential kinematics
   |
   v
MuJoCo 四轮执行器 / Four wheel actuators
   |
   +----> /ground_truth/odom
   +----> /clock
   +----> /tf
```

功能包使用 `ament_python`，模型、配置与 launch 文件均安装到包共享目录。运行时通过 ROS 2 包索引定位资源，不依赖用户目录、固定工作空间路径或网络地址。

The package uses `ament_python`; its model, configuration, and launch files are installed into the package share directory. Runtime resource lookup uses the ROS 2 package index and does not depend on a user directory, a fixed workspace path, or a network address.

## 环境与构建 / Environment and Build

目标环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10 和 MuJoCo 3.12.0。

The target environment is Ubuntu 22.04, ROS 2 Humble, Python 3.10, and MuJoCo 3.12.0.

```bash
python3 -m pip install -r src/ground/mujoco_diff_drive_slip_compensation/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_diff_drive_slip_compensation
source install/setup.bash
```

## 启动与检查 / Launch and Inspect

```bash
ros2 launch mujoco_diff_drive_slip_compensation main.launch.py
ros2 topic list
ros2 topic echo /ground_truth/odom
```

另一个终端可发送速度指令：

Send a velocity command from another terminal:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.25}}"
```

## 参数 / Parameters

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
| --- | ---: | --- |
| `model_file` | `models/diff_drive.xml` | 包共享目录内的 MJCF 路径 / MJCF path below the package share directory |
| `wheel_radius` | `0.08` | 车轮半径，单位 m / wheel radius in metres |
| `track_width` | `0.42` | 左右轮中心距，单位 m / lateral wheel separation in metres |
| `max_wheel_speed` | `20.0` | 最大轮速，单位 rad/s / maximum wheel speed in rad/s |
| `command_timeout` | `0.5` | 自动停车超时，单位 s / automatic stop timeout in seconds |
| `publish_tf` | `true` | 是否发布坐标变换 / whether to publish transforms |

## 安全边界 / Safety Boundaries

- 指令含非有限数值时拒绝更新。 / Reject commands containing non-finite values.
- 指令超时后轮速归零。 / Set wheel speeds to zero after command timeout.
- 四轮目标速度按执行器上限等比例缩放。 / Scale all wheel targets proportionally to actuator limits.
- 包资源路径经过边界检查。 / Validate package-resource paths against directory escape.

## 验证命令 / Verification Commands

```bash
python3 -m pytest src/ground/mujoco_diff_drive_slip_compensation/test -q
python3 -m compileall src/ground/mujoco_diff_drive_slip_compensation
colcon test --packages-select mujoco_diff_drive_slip_compensation
colcon test-result --verbose
```
