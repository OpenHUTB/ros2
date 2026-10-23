# 无人机轻量神经网络与 ROS 2 接口 / Lightweight UAV neural control and ROS 2 interface

## 目标 / Objective

本页记录前向测距制动模块的学习控制与 ROS 2 封装。/ This page records the learning controller and ROS 2 wrapper for the forward-ranging braking module.

## 学习控制 / Learning controller

训练脚本读取三个数据划分，使用 `sensor_range_m` 与 `closing_speed_m_s` 预测解析控制器的加速度标签。/ The training script reads the three dataset splits and predicts the analytical controller's acceleration target from `sensor_range_m` and `closing_speed_m_s`.

模型是 NumPy 实现的两层 MLP：16 个 ReLU 隐藏单元，固定随机种子 2026，权重保存在 `models/mlp_controller.json`。/ The model is a NumPy two-layer MLP with 16 ReLU units and seed 2026; weights are stored in `models/mlp_controller.json`.

当前测试集 MAE 为 0.1454 m/s²，RMSE 为 0.2604 m/s²。默认场景使用 MLP 运行 1000 步，碰撞步数为 0，最小间距为 0.459629 m。/ Current test MAE is 0.1454 m/s² and RMSE is 0.2604 m/s². The default MLP run completes 1000 steps with zero collision steps and 0.459629 m minimum clearance.

## ROS 2 节点 / ROS 2 node

`ros_node.py` 以固定周期推进同一 MuJoCo 仿真，并发布以下话题：/ `ros_node.py` advances the same MuJoCo simulation on a fixed timer and publishes:

| 话题 / Topic | 消息 / Message | 内容 / Content |
| --- | --- | --- |
| `/uav/front_range_m` | `std_msgs/Float32` | 前向测距（米） / Forward range in metres |
| `/uav/braking_accel_m_s2` | `std_msgs/Float32` | 水平控制加速度 / Horizontal control acceleration |
| `/uav/state` | `std_msgs/Float64MultiArray` | 时间、测距、闭合速度、高度 / Time, range, closing speed, height |

Ubuntu 22.04 / ROS 2 Humble 中可运行：/ Run on Ubuntu 22.04 / ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ./launch/ros_node.launch.py controller:=mlp
ros2 topic echo /uav/state
```

该节点用于模块级闭环观察和话题联调；当前模型固定航向，未模拟姿态、电机和真实飞控接口。/ The node supports module-level closed-loop observation and topic integration; the current model has fixed heading and does not model attitude, motors, or a real flight-controller interface.
