# 麦克纳姆轮全向移动平台 8字轨迹追踪仿真

## 功能介绍

本模块实现四轮麦克纳姆轮全向移动平台的运动学建模、Gazebo仿真环境搭建，以及基于PD控制器的8字（李萨如）轨迹追踪仿真。

通过本示例可以学习：
- 麦克纳姆轮全向移动底盘的运动学原理
- 如何在URDF中定义四轮移动机器人
- Gazebo ros_control 速度控制器配置
- 8字（李萨如）参数轨迹生成
- 基于PD的轨迹追踪控制

## 运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04 LTS |
| ROS版本 | Noetic |
| 仿真器 | Gazebo 11 |
| 内存要求 | 至少4GB |

## 运行步骤

### 1. 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 基础仿真（直线前进）

```bash
roslaunch mecanum_8shape_trajectory base_sim.launch
# 新开终端运行入口脚本
rosrun mecanum_8shape_trajectory main.py
```

### 3. 8字轨迹追踪（一键启动）

```bash
roslaunch mecanum_8shape_trajectory trajectory_track.launch
```

## 运行效果

![8字轨迹仿真效果](images/trajectory_8shape.gif)

## 参考链接

- [运动学推导](kinematics.md)
- [仿真配置说明](simulation.md)
