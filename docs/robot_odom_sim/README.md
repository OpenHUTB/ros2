# 基于 ROS TF 树与轮式里程计的机器人速度积分仿真

## 项目介绍
本项目在 ROS 底层实现了一个基于速度指令（Twist）直接积分的轮式里程计仿真节点。节点接收键盘发布的 `/cmd_vel` 速度指令，通过前向欧拉积分计算得出机器人位姿，并实时发布 `nav_msgs/Odometry` 消息以及 `/odom` 到 `/base_link` 的 TF 坐标变换。加入超时保护，无指令时自动停止。

## 运行环境
- Ubuntu 20.04
- ROS Noetic
- Python 3.8

## 文件结构
```
src/robot_odom_sim/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── robot_odom.launch
├── rviz/
│   └── robot_odom.rviz
└── scripts/
├── main.py
└── teleop_test.py

docs/robot_odom_sim/
├── README.md
└── robot_odom.gif
```

## 运行步骤
1. 编译并加载环境：`catkin_make` 和 `source devel/setup.bash`
2. 一键启动仿真、键盘控制与 Rviz：`roslaunch robot_odom_sim robot_odom.launch`
3. 在弹出的键盘控制终端中，使用 W/A/S/D 控制机器人在 Rviz 中移动。

## 运行结果
*(这里放你压缩到 5MB 以下的动图)*

## 大模型使用声明
代码与文档编写过程中使用了大模型进行辅助生成，核心逻辑与测试已由本人独立完成并验证。

