ROS 初次实践——第一个 ROS 例程（小海龟仿真）本项目基于 ROS 机器人操作系统的 turtlesim 核心仿真器开发，涵盖节点启动、键盘控制、话题发布、服务调用，以及编写 Python 控制节点驱动海龟进行指定轨迹运动的完整实验流程。实验环境操作系统：Ubuntu 20.04 LTS / 22.04 LTSROS 版本：ROS 1 (Noetic)核心依赖包：turtlesim、geometry_msgs、rospy核心实验步骤1. 基础节点启动与键盘控制按顺序在三个独立终端执行以下命令：终端 1：启动 ROS MasterBashroscore
终端 2：启动小海龟仿真器节点Bashrosrun turtlesim turtlesim_node
终端 3：启动键盘遥控节点Bashrosrun turtlesim turtle_teleop_key
2. 命令行话题与服务调试查看当前运行的节点列表：Bashrosnode list
命令行发布速度话题（以 10Hz 频率持续驱动海龟）：Bashrostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
发送服务请求（生成新海龟）：Bashrosservice call /spawn "x: 5.0
y: 5.0
theta: 0.0
name: 'turtle2'"
核心控制源码 (turtle_controller.py)Python#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def turtle_circle_publisher():
    rospy.init_node('turtle_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    rospy.loginfo("Turtle controller node has started, moving the turtle in a circle...")

    vel_msg = Twist()
    vel_msg.linear.x = 2.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 1.8

    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        turtle_circle_publisher()
    except rospy.ROSInterruptException:
        pass
节点与通信机制通信要素名称 / 命令接口类型说明仿真节点/turtlesimturtlesim_node图形界面与物理仿真引擎控制节点/teleop_turtleturtle_teleop_key捕获键盘按键并发送控制消息自主节点/turtle_controller自定义 Python 脚本周期发布速度指令驱动海龟控制话题/turtle1/cmd_velgeometry_msgs/Twist传输线速度与角速度状态话题/turtle1/poseturtlesim/Pose实时反馈坐标与姿态角生成服务/spawnturtlesim/Spawn动态实例化新海龟实验效果展示阶段一：仿真器启动与键盘遥控阶段二：话题调试与服务调用作者信息