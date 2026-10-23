# 多目标点导航（turtlebot_multi_goal_nav）

`turtlebot_multi_goal_nav` 是一个多目标点导航的 ROS1 包，用 TurtleBot3（Gazebo 仿真）按顺序走
5 个目标点，到点就切下一个，全走完自动停车。正前方障碍太近时会停车、原地右转躲一下再继续。

源码在 `src/ground/turtlebot_multi_goal_nav/`，入口是 `main.py`。

## 它怎么工作的

节点订阅 `/odom` 拿到小车的位置和朝向，算出到当前目标点的距离和角度差，再发布 `/cmd_vel` 控制小车：
角度差大就先原地转向对准目标方向，对准了就直行。同时订阅 `/scan` 看正前方的距离，太近了就停车右转躲一下。

目标点是这 5 个（单位米）：(1.5,0)、(1.5,3)、(-2.5,3)、(-2.5,-1.5)、(0,0)，最后一个回到出发点。

## 运行

环境、编译和启动步骤见模块里的 [`README.md`](https://github.com/OpenHUTB/ros2/blob/master/src/ground/turtlebot_multi_goal_nav/README.md)。

快速启动：

```bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot_multi_goal_nav turtlebot_multi_goal_nav.launch
```

## 运行效果

![运行效果](../img/ground/turtlebot_multi_goal_nav.gif)

## 验证记录

在自己的机器上实际构建、启动、运行过：Ubuntu 20.04.6 LTS（focal）、ROS Noetic、Python 3.8.10、
catkin 0.8.12、Gazebo 11.15.1、TurtleBot3 burger。`catkin_make` 编译通过，仿真和节点正常启动，
小车按顺序走完 5 个目标点并自动停车，终端输出“到达目标 1～5”后打印“全部目标点跑完，停车！”。

## 声明

`main.py` 是作业的原始代码，其余文件（`package.xml`、`CMakeLists.txt`、`launch`、`README.md`、本文档）在整理时用到了大模型辅助。我已逐项核对，对提交内容负责。
