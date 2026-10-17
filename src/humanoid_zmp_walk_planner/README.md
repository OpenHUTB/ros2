# 人形机器人 - 基于 ZMP 的步行轨迹规划 ROS 2 封装模块

本模块基于简化的人形机器人 ZMP（零力矩点）运动学模型，在 ROS 2 环境下实现了一个步行轨迹规划与可视化节点。节点模拟了人形机器人一个完整步态周期（支撑期与摆动期）的足部轨迹，并实时绘制出机器人的运动姿态，验证了双足行走的步态规划逻辑。

## 运行环境
* 操作系统：Ubuntu 22.04（虚拟机）
* ROS 2 版本：Humble
* 依赖库：rclpy, geometry_msgs, matplotlib, numpy

## 核心代码文件清单

本模块包含以下核心文件，均已提交至仓库：

```text
src/
└── humanoid_zmp_walk_planner/
    ├── main.py                 # ZMP 步行轨迹规划与可视化节点
    ├── setup.py                # ROS 2 功能包配置文件
    ├── package.xml             # ROS 2 依赖声明文件
    ├── README.md               # 模块说明文档
    └── launch/
        └── main.launch.py      # 一键启动文件
```
## 运行步骤

1. 新建标准工作空间并进入 src 目录：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. 从 ros2 仓库拷贝模块代码到工作空间：

```bash
cp -r ~/OpenHUTB/ros2/src/humanoid_zmp_walk_planner ~/ros2_ws/src/
cd ~/ros2_ws
```

3. 编译工作空间并部署环境：

```bash
colcon build --packages-select humanoid_zmp_walk_planner
source install/setup.bash
```

4. 运行 ZMP 步行轨迹规划节点：

```bash
ros2 launch humanoid_zmp_walk_planner main.launch.py
```


> 📖 **详细原理与实验步骤请参阅：** [点击查看主文档](../../docs/humanoid/zmp_walk_planner.md)
