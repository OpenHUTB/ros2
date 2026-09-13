#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 小海龟画正方形节点：turtle_square_node
# 思路：直走一条边 -> 停 0.5 秒 -> 左转 90 度 -> 停 0.5 秒，循环 4 次就是正方形
#
# 重点：turtlesim 内置 0.5 秒看门狗，一旦超过 0.5 秒没收到新的 cmd_vel 消息
# 就会强制把海龟刹停。如果只在动作开始时发一条速度指令，海龟会被中途刹车，
# 导致边长不足、转角不足 90 度。
# 因此本节点用 rospy.Rate(10) 以 10 Hz 高频持续发布速度指令，直到动作结束。

import rospy
from geometry_msgs.msg import Twist


class SquareDrawer:
    """控制小海龟画边长 2 m 的正方形"""

    def __init__(self):
        # 初始化节点
        rospy.init_node('turtle_square_node')
        # 发布者：把速度指令发到 /turtle1/cmd_vel 话题
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # 10 Hz 高频持续发布，防止 turtlesim 0.5 秒超时强制刹车
        self.rate = rospy.Rate(10)

    def publish_for(self, twist, duration):
        """在 duration 秒内以 10 Hz 持续发布 twist，时间一到就结束"""
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.pub.publish(twist)
            self.rate.sleep()

    def stop(self, duration=0.5):
        """持续发布空速度 0.5 秒：让海龟停下，并把残余惯性消掉"""
        self.publish_for(Twist(), duration)

    def run(self):
        # 直行指令：线速度 1.0 m/s
        straight = Twist()
        straight.linear.x = 1.0
        # 转弯指令：角速度 1.0 rad/s，绕 z 轴逆时针旋转即左转
        turn = Twist()
        turn.angular.z = 1.0

        # 先等 1 秒，让发布者和 turtlesim 的订阅端连好，避免第一条消息丢失
        rospy.sleep(1.0)

        rospy.loginfo("开始画正方形，边长 2 米")
        for i in range(4):
            # 直行：1.0 m/s x 2 s = 2 m
            rospy.loginfo("正在画第 %d 条边", i + 1)
            self.publish_for(straight, 2.0)
            self.stop()

            # 左转 90 度：1.0 rad/s x 1.57 s ≈ π/2
            rospy.loginfo("正在转第 %d 个角", i + 1)
            self.publish_for(turn, 1.57)
            self.stop()

        rospy.loginfo("正方形画完啦！")


if __name__ == '__main__':
    node = SquareDrawer()
    try:
        node.run()
    except rospy.ROSInterruptException:
        # 按 Ctrl+C 或者节点被关掉的时候走到这里
        rospy.loginfo("节点被中断，让海龟停下来")
    finally:
        # 不管正常画完还是中间出错，最后都再发一次 0 速度，保险一点
        if not rospy.is_shutdown():
            node.pub.publish(Twist())
