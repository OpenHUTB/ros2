# 新增无人机飞行状态识别与轨迹绘制 ROS 2 封装模块
本模块基于 ROS 2 实现了一套无人机飞行状态识别系统。系统通过发布节点模拟无人机速度指令（Twist），接收节点实时订阅数据，自动识别当前的飞行状态（直线、旋转、悬停），并基于固定步长模拟实时绘制飞行轨迹图。

## 运行环境
* 操作系统：Ubuntu 22.04（虚拟机）
* ROS 2 版本：Humble
* 依赖库：rclpy, geometry_msgs, matplotlib
## 核心代码文件清单

本模块包含以下核心文件，均已提交至仓库：

```text
src/drone_trajectory_logger/
├── drone_trajectory_logger/
│   ├── __init__.py
│   ├── main.py
│   └── trajectory_analyzer.py
├── resource/
│   └── drone_trajectory_logger
├── launch/
│   └── main.launch.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 快速开始
```bash
colcon build --packages-select drone_trajectory_logger
source install/setup.bash
ros2 launch drone_trajectory_logger main.launch.py
```
> 📖 **详细原理与实验步骤请参阅：** [点击查看主文档](../../docs/air/drone_trajectory_logger.md)

