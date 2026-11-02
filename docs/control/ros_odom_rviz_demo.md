# ROS 虚拟机器人圆形轨迹 RViz 可视化入门

## 一、项目概述

本项目基于 ROS Noetic，编写了一个简单的 ROS 节点，持续发布里程计（Odometry）话题。该节点模拟了一个虚拟机器人沿着半径 1 米的圆形轨迹匀速绕圈，并通过 RViz 进行实时可视化展示。

**项目亮点：**
- 轻量级：无需安装 Gazebo 等大型仿真软件，基础 ROS 环境即可运行。
- 直观：一键启动后，RViz 自动加载预配置视图，直观看到小机器人绕圈。
- 知识点串联：将“发布消息”与“RViz 可视化”结合，适合初学者练手。

## 二、部署与运行流程

### 2.1 创建并编译工作区
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
