# 人形机器人 - 简化双足步态动画演示 ROS 2 封装模块

## 作业概述与背景
本模块实现简化双足步态演示，在 ROS 2 环境下实现了一个步行轨迹规划可视化节点。节点模拟了人形机器人一个完整步态周期（支撑期与摆动期）的腿部轨迹，并实时绘制出机器人的运动姿态，完成双足行走动画仿真。

## 运行环境
* 操作系统：Ubuntu 22.04（虚拟机）
* ROS 2 版本：Humble
* 依赖库：rclpy, matplotlib

## 核心代码文件清单

本模块包含以下核心文件，均已提交至仓库：

```text
src/
└── humanoid_zmp_walk_planner/
    ├── humanoid_zmp_walk_planner/        # Python 源码目录
    │   ├── __init__.py                   # 空文件，标识包 
    │   └── main.py                       # 简化双足步态动画可视化节点
    ├── resource/                         # 资源目录
    │   └── humanoid_zmp_walk_planner     # 无后缀空文件
    ├── launch/
    │   └── main.launch.py                # 一键启动文件
    ├── package.xml                       # ROS 2 依赖声明文件
    ├── setup.py                          # ROS 2 功能包配置文件
    ├── setup.cfg                         # 脚本安装路径配置
    └── README.md                         # 模块说明文档
```

## 运行步骤

1. 新建标准工作空间并进入 src 目录：

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. 从 ros2 仓库拷贝模块代码到工作空间：

```
cp -r ~/OpenHUTB/ros2/src/humanoid_zmp_walk_planner ~/ros2_ws/src/
cd ~/ros2_ws
```

3. 编译工作空间并部署环境：

```
colcon build --packages-select humanoid_zmp_walk_planner
source install/setup.bash
```

4. 启动简化双足步态动画演示与可视化节点：

```
ros2 launch humanoid_zmp_walk_planner main.launch.py
```

> 📖 **详细原理与实验步骤请参阅：** [点击查看主文档](../../docs/humanoid/zmp_walk_planner.md)
