# 第5章 机器人感知

本章介绍基于 ROS 2 与神经网络的多模态传感器感知模块。每个模块都自包含源代码、训练脚本、Gazebo 仿真场景与 launch 入口，并满足 OpenHUTB/ros2 仓库的提交约定（`main.py` 入口、launch 启动、全英文文件名、`docs/` + `src/` 目录约定）。

## 已收录模块

- [基于激光雷达扫描的神经网络障碍物分类](chap5/lidar_obstacle_nn.md)
  - 仿真器：Gazebo Fortress 6.x
  - 传感器：360 束 2D GPU LiDAR（`sensor_msgs/LaserScan`）
  - 神经网络：三层 PyTorch MLP（360→128→128→3）
  - 任务：把每条扫描分类为 `near` / `far` / `free` 三类
  - 源代码：[`src/chap5/lidar_obstacle_nn/`](https://github.com/OpenHUTB/ros2/tree/master/src/chap5/lidar_obstacle_nn)

## 后续规划模块（占位）

以下方向待补，作为后续 PR 的增量提交：

- 相机目标检测（YOLOv8 + Gazebo 仿真相机）
- 深度相机避障 NN（深度图 → 速度指令）
- 激光雷达 + 相机融合感知
- 感知→运动控制闭环（感知结果驱动 cmd_vel）
