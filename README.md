ROS 初次实践——第一个 ROS 例程（小海龟仿真）
本项目基于 ROS 机器人操作系统的 turtlesim 核心仿真器开发，涵盖节点启动、键盘控制、话题发布、服务调用，以及编写 Python 控制节点驱动海龟进行指定轨迹运动的完整实验流程。
一.实验环境
1.操作系统：Ubuntu 20.04 LTS / 22.04 LTS
2.ROS 版本：ROS 1 (Noetic)
3.核心依赖包：turtlesim、geometry_msgs、rospy
二.核心实验步骤
1. 基础节点启动与键盘控制按顺序在三个独立终端执行以下命令：
   终端 1：启动 ROS Master
               Bash
               roscore
   终端 2：启动小海龟仿真器节点
               Bash
               rosrun turtlesim turtlesim_node
   终端 3：启动键盘遥控节点
               Bash
               rosrun turtlesim turtle_teleop_key
2. 命令行话题与服务调试
           查看当前运行的节点列表：
               Bash
               rosnode list
           命令行发布速度话题（以 10Hz 频率持续驱动海龟）：
               Bash
               rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "linear:
                 x: 1.0
                 y: 0.0
                 z: 0.0
               angular:
                 x: 0.0
                 y: 0.0
                 z: 0.0"
3.发送服务请求（生成新海龟）：
               Bash
               rosservice call /spawn "x: 5.0
               y: 5.0
               theta: 0.0
               name: 'turtle2'"
4.核心控制源码 (turtle_controller.py)
  Python
  #!/usr/bin/env python3
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
实验效果展示
阶段一：仿真器启动与键盘遥控
阶段二：话题调试与服务调用作者信息
