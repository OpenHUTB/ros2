# ROS基础实验作业 README
## 一、作业概述
本次作业完成 **ROS环境部署、Turtlesim小海龟仿真实验、ROS基础命令实操**，并拓展学习 Carla-ROS 桥接源码与 C++ 实现逻辑，掌握 ROS 节点、话题、消息通信的基础原理，熟悉机器人仿真开发基本流程。

实验任务清单：
1. 安装 ROS 环境（Kinetic/Indigo）
2. 运行小海龟仿真，实现键盘控制运动
3. 通过仿真器熟练掌握常用 ROS 终端命令
4. 研读、注释 Carla-ROS Bridge C++ 源码，理解桥接原理

## 二、实验环境
- 操作系统：Ubuntu 16.04（适配 ROS Kinetic）
- ROS 版本：ROS Kinetic Kame
- 仿真工具：turtlesim 官方仿真器
- 拓展环境：Carla 仿真器 + ROS Bridge C++ 源码

## 三、实验一：ROS 环境安装
### 1. 安装流程
按照官方教程完成软件源更换、密钥添加、完整桌面版 ROS 安装、rosdep 初始化、环境变量配置，最终将 ROS 环境永久写入 bash 终端配置文件。
### 2. 环境验证
终端输入以下命令启动 ROS 核心节点：
```bash
roscore
