# 水下机器人 MuJoCo 仿真 — ROS2 功能包

## 概述

本功能包将 MuJoCo 物理引擎中的水下机器人（ROV + UR5e 机械臂）仿真封装为 ROS2 节点，支持通过标准 ROS2 话题进行控制和数据获取。

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- MuJoCo 3.12.0

## 安装

```bash
# 进入 ROS2 工作空间
cd ~/ros2_ws/src

# 复制本功能包
cp -r /mnt/e/课程作业/机器人操作系统及应用/rov_mujoco .

# 编译
cd ~/ros2_ws
colcon build --packages-select rov_mujoco --symlink-install
source install/setup.bash
```

## 运行

### 6-DOF 键盘运动控制

```bash
ros2 launch rov_mujoco main.launch.py
```

### 键盘操作

| 按键 | 功能 |
|------|------|
| W/S | 前进/后退 |
| A/D | 左移/右移 |
| Q/E | 上浮/下潜 |
| J/L | 左偏航/右偏航 |
| I/K | 前俯仰/后俯仰 |
| U/O | 左翻滚/右翻滚 |
| 空格 | 急停 |
| +/- | 调节速度比例 |

## ROS2 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令（订阅） |
| `/rov/odom` | `nav_msgs/Odometry` | 里程计（发布） |
| `/rov/depth` | `std_msgs/Float64` | 深度（发布） |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态（发布） |
