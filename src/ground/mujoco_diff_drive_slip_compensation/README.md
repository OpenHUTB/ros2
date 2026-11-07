# MuJoCo 差速小车轮滑感知与轨迹补偿

MuJoCo Differential-Drive Slip Perception and Trajectory Compensation

本 ROS 2 功能包提供一个四轮差速小车的 MuJoCo 仿真基础，用于后续接入编码器、惯性测量单元、轮滑感知与轨迹补偿。当前入口接收 `cmd_vel`，通过差速运动学计算四轮目标转速，并发布仿真时钟、真实里程计和坐标变换。

This ROS 2 package provides a MuJoCo simulation foundation for a four-wheel differential-drive robot. It is designed for subsequent encoder, inertial measurement unit, slip-perception, and trajectory-compensation work. The current entry point consumes `cmd_vel`, converts chassis commands to four wheel-speed targets, and publishes simulation time, ground-truth odometry, and transforms.

## 环境 / Environment

| 组件 / Component | 版本 / Version |
| --- | --- |
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Python | 3.10 |
| MuJoCo | 3.12.0 |
| PyTorch | CPU build, required by later training work / 后续训练阶段使用 CPU 版本 |

## 目录 / Layout

```text
mujoco_diff_drive_slip_compensation/
├── config/simulation.yaml
├── launch/main.launch.py
├── models/diff_drive.xml
├── mujoco_diff_drive_slip_compensation/
│   ├── kinematics.py
│   ├── main.py
│   ├── resources.py
│   └── simulator.py
├── test/
├── package.xml
├── requirements.txt
├── setup.cfg
└── setup.py
```

## 安装与构建 / Install and Build

在 ROS 2 工作空间根目录执行：

Run the following commands from the ROS 2 workspace root:

```bash
python3 -m pip install -r src/ground/mujoco_diff_drive_slip_compensation/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_diff_drive_slip_compensation
source install/setup.bash
```

## 运行 / Run

```bash
ros2 launch mujoco_diff_drive_slip_compensation main.launch.py
```

发送 0.4 m/s 的前进指令：

Send a 0.4 m/s forward command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

## 接口 / Interfaces

| 方向 / Direction | 名称 / Name | 类型 / Type | 说明 / Description |
| --- | --- | --- | --- |
| 订阅 / Subscribe | `/cmd_vel` | `geometry_msgs/msg/Twist` | 车体速度指令 / chassis velocity command |
| 发布 / Publish | `/ground_truth/odom` | `nav_msgs/msg/Odometry` | MuJoCo 真实状态 / MuJoCo ground truth |
| 发布 / Publish | `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟 / simulation clock |
| 发布 / Publish | `/tf` | `tf2_msgs/msg/TFMessage` | `odom` 到 `base_link` / `odom` to `base_link` |

速度指令超过 `command_timeout` 未更新时，节点将四轮目标速度清零。非有限数值和越出包资源目录的模型路径会被拒绝。

When no velocity command arrives within `command_timeout`, the node sets all wheel targets to zero. Non-finite commands and model paths that escape the package share directory are rejected.

## 测试 / Test

```bash
python3 -m pytest src/ground/mujoco_diff_drive_slip_compensation/test -q
python3 -m compileall src/ground/mujoco_diff_drive_slip_compensation
```

ROS 2 环境中的完整验收还应包含 `colcon test` 与启动后的话题检查。

Full acceptance in a ROS 2 environment also includes `colcon test` and topic inspection after launch.
