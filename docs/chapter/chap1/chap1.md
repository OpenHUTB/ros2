# 第1章 认识 ROS

本章通过 ROS Noetic 中的 turtlesim 小海龟仿真学习 ROS 的基本使用方法，
包括话题通信、速度控制、位姿反馈以及服务调用。

## 实验内容

### 1. 小海龟绘制正方形

通过向 `/turtle1/cmd_vel` 发布 `geometry_msgs/Twist` 消息，
控制小海龟完成直线运动和转向，从而自主绘制正方形。

[进入小海龟画正方形实验](turtle_square.md)

### 2. 小海龟绘制圆形

同时向 `/turtle1/cmd_vel` 发布线速度和角速度，
并通过 `/turtle1/pose` 的位姿反馈判断是否完成一圈。

[进入小海龟画圆实验](turtle_circle.md)

### 3. 小海龟绘制 OpenHUTB

订阅 `/turtle1/pose` 获取实时位姿，并结合 `/turtle1/cmd_vel`、
`/turtle1/set_pen` 和 `/turtle1/teleport_absolute`，
控制小海龟自动绘制 OpenHUTB 字样。

[进入小海龟绘制 OpenHUTB 实验](turtle_hutb.md)

### 4. ROS2 Service 小海龟距离控制

通过 ROS2 Service 控制 turtlesim 小海龟按照指定距离移动，并根据 `/turtle1/pose` 的位置反馈判断实际移动距离。

[进入 ROS2 Service 小海龟距离控制实验](move_forward.md)

### 5. 小海龟画花瓣

综合 `/set_pen` 画笔服务、`/teleport_absolute` 瞬移服务与速度话题发布，
控制小海龟画出 6 个圆心均匀分布、两两相扣的彩色花瓣。

[进入小海龟画花瓣实验](turtle_sim_experiment.md)


### 6. 小海龟画五角星

使用发布器向`/turtle1/cmd_vel`话题发布速度指令，使用`/turtle1/set_pen`服务动态修改画笔RGB颜色，实现彩色绘图，通过定时方式控制海龟直行距离与旋转角度，完成五角星轨迹绘制，使用`rosnode`、`rostopic`、`rosservice`工具查看节点、话题、服务，验证ROS通信。

[进入小海龟画五角星实验](turtle_star.md)

### 8. 小海龟画爱心

通过爱心参数方程生成轨迹点，控制小海龟绘制出一个完整的爱心图案。

[进入小海龟画爱心实验](turtle_heart.md)
