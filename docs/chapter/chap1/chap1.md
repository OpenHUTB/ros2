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

### 4. 小海龟画花瓣（服务调用与 ROS 命令实践）

先用键盘遥控和 rosnode、rostopic、rosservice、rosparam 等命令
熟悉小海龟系统的操作与调试，再通过 `/spawn` 服务生成第二只海龟，
综合 `/set_pen`、`/teleport_absolute` 服务与话题发布，
让 turtle2 自动画出 6 个两两相扣的彩色花瓣。

[进入小海龟画花瓣实验](turtle_sim_experiment.md)
