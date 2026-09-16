#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""小海龟自动画花瓣演示节点（配套文档：docs/chapter/turtle_sim_experiment.md）

工作流程：
  1. 等待 turtlesim 的 /spawn 服务上线；
  2. 生成第二只海龟 turtle2；
  3. 让 turtle2 依次画出 petals 个圆：每个圆的圆心均匀分布在公共中心四周，
     且圆心到公共中心的距离正好等于圆的半径，所以每个圆都经过公共中心，
     画出来就是一圈两两相扣的"花瓣"；
  4. 每个花瓣换一种画笔颜色。

兼容说明：采用 Python 2/3 兼容写法，ROS Kinetic（Python 2.7）与 Noetic（Python 3）均可运行。
可调参数（rosrun/roslaunch 参数服务器）：~petals 花瓣数、~linear_speed 线速度、~angular_speed 角速度。
"""
from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute

# 花瓣颜色表（r, g, b），按顺序循环取用
PETAL_COLORS = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
                (255, 255, 0), (255, 0, 255), (0, 255, 255)]

# 公共中心：所有花瓣圆都经过这一点
CENTER_X, CENTER_Y = 5.5, 5.5


def main():
    rospy.init_node('turtle_circle_drawer')

    petals = rospy.get_param('~petals', 6)                  # 画几个花瓣
    linear_speed = rospy.get_param('~linear_speed', 1.5)    # 线速度 m/s
    angular_speed = rospy.get_param('~angular_speed', 1.0)  # 角速度 rad/s
    radius = linear_speed / angular_speed                   # 圆半径 r = v / ω
    circle_period = 2.0 * math.pi / angular_speed           # 画整圆用时 T = 2π / ω

    # 1. 同步等待 turtlesim 的服务上线，避免本节点早于仿真器启动而调用失败
    rospy.loginfo('等待 /spawn 服务上线 ...')
    rospy.wait_for_service('/spawn')
    spawn = rospy.ServiceProxy('/spawn', Spawn)
    set_pen = rospy.ServiceProxy('/turtle2/set_pen', SetPen)
    teleport = rospy.ServiceProxy('/turtle2/teleport_absolute', TeleportAbsolute)

    # 2. 生成第二只海龟 turtle2（返回值里带回实际名字，默认不会重名）
    resp = spawn(CENTER_X, CENTER_Y, 0.0, 'turtle2')
    rospy.loginfo('已生成小海龟: %s', resp.name)

    # 3. 以 50 Hz 向 /turtle2/cmd_vel 发布速度指令。
    #    turtlesim 有 0.5 秒看门狗：超时收不到指令就会把海龟刹停，
    #    因此必须用循环高频发布，而不能只发一条。
    pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed

    try:
        for i in range(petals):
            # 每个花瓣对应一个圆：圆心在"公共中心 + r*u(theta)"处（u 为单位向量），
            # 起点取圆心再向外挪 r，即"公共中心 + 2r*u(theta)"；
            # 起点处圆的切线方向与半径方向垂直，所以朝向设为 theta + 90 度。
            theta = 2.0 * math.pi * i / petals
            start_x = CENTER_X + 2.0 * radius * math.cos(theta)
            start_y = CENTER_Y + 2.0 * radius * math.sin(theta)
            heading = theta + math.pi / 2.0

            r, g, b = PETAL_COLORS[i % len(PETAL_COLORS)]
            rospy.loginfo('开始画第 %d 个花瓣，画笔 RGB=(%d, %d, %d)', i + 1, r, g, b)

            set_pen(r, g, b, 3, 1)               # 先抬笔，瞬移过程不画线
            teleport(start_x, start_y, heading)
            set_pen(r, g, b, 3, 0)               # 落笔开画

            end_time = rospy.Time.now() + rospy.Duration(circle_period)
            while rospy.Time.now() < end_time and not rospy.is_shutdown():
                pub.publish(twist)
                rate.sleep()

            set_pen(r, g, b, 3, 1)               # 画完抬笔
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo('演示完成：turtle2 共画出 %d 个半径 %.2f 米的花瓣圆', petals, radius)


if __name__ == '__main__':
    main()
