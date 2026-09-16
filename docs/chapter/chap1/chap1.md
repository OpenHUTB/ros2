# 第1章 认识 ROS

本章通过 ROS Noetic 中的 turtlesim 小海龟仿真学习 ROS 的基本使用方法，
包括话题通信、速度控制、位姿反馈以及服务调用。

## 实验内容

### 1. 小海龟绘制正方形

通过向 `/turtle1/cmd_vel` 发布 `geometry_msgs/Twist` 消息，
控制小海龟完成直线运动和转向，从而自主绘制正方形。

[进入小海龟画正方形实验](turtle_square.md)

### 2. 小海龟绘制 OpenHUTB

订阅 `/turtle1/pose` 获取实时位姿，并结合 `/turtle1/cmd_vel`、
`/turtle1/set_pen` 和 `/turtle1/teleport_absolute`，
控制小海龟自动绘制 OpenHUTB 字样。

[进入小海龟绘制 OpenHUTB 实验](turtle_hutb.md)

### 3. ROS2 Service 小海龟距离控制

通过 ROS2 Service 控制 turtlesim 小海龟按照指定距离移动，并根据 `/turtle1/pose` 的位置反馈判断实际移动距离。

[进入 ROS2 Service 小海龟距离控制实验](move_forward.md)
