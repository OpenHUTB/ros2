# 单自由度机械臂仿真

## 模块介绍
本模块实现了一个单自由度旋转关节机械臂的 Gazebo 仿真，包括 URDF 建模、位置控制器配置和关节角度自动演示。

## 功能特性
- 单旋转关节机械臂模型
- PID 位置控制
- Gazebo 物理仿真
- 关节角度自动演示（90° → 0° → -90° → 0°）

## 运行环境
- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11

## 快速开始

### 编译
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
