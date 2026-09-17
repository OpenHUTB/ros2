\# ROS2 小海龟画圆实验

\## 实验简介

本实验基于ROS2，编写Python节点，持续向`/turtle1/cmd\\\_vel`话题发布速度指令，控制turtlesim仿真器中的海龟，实现圆周运动。



\## 环境

\- ROS2 版本：Humble

\- 编程语言：Python3

\- 依赖包：`rclpy`、`geometry\\\_msgs`、`turtlesim`



\## 文件说明

\- `draw\\\_circle.py`：海龟画圆核心代码，创建节点，定时发布速度消息。



\## 运行步骤

1\. 启动海龟仿真器

```bash

ros2 run turtlesim turtlesim\\\_node


