#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
节点1: 速度指令控制器 (cmd_vel_publisher)
发布话题: /cmd_vel   (geometry_msgs/Twist)
作用   : 周期性地发布线速度 v 和角速度 w, 驱动机器人沿"方形"路径行走。
         直走 3.2 秒 -> 原地转弯 1.5 秒 -> 循环。
运行(手动):  rosrun humanoid_base_sim cmd_vel_publisher.py
"""
import rospy
from geometry_msgs.msg import Twist


class CmdVelPublisher:
    def __init__(self):
        # 话题名 cmd_vel, 消息类型 Twist, 队列长度 1(只关心最新一条指令)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)                     # 10 Hz
        self.t0 = rospy.Time.now().to_sec()            # 启动时刻(秒)

        # 路径参数
        self.straight_t = 3.2      # 每次直走时长(秒)
        self.turn_t     = 1.5      # 每次原地转弯时长(秒)
        self.straight_v = 0.55     # 直走线速度 m/s
        self.turn_w     = 0.55     # 转弯角速度 rad/s

    def run(self):
        rospy.loginfo("[cmd_vel_publisher] 开始发布 cmd_vel ...")
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - self.t0
            phase = t % (self.straight_t + self.turn_t)

            msg = Twist()
            if phase < self.straight_t:
                msg.linear.x = self.straight_v     # 直走
            else:
                msg.angular.z = self.turn_w        # 原地转弯
            self.pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("cmd_vel_publisher")
    CmdVelPublisher().run()
