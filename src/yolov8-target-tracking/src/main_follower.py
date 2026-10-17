#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray # 引入接收坐标的消息类型

class TargetFollower:
    def __init__(self):
        # 创建一个发布者，用来控制小车移动
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 核心改动：不再订阅图像！而是订阅眼睛发来的坐标话题 /target_position
        self.sub = rospy.Subscriber("/target_position", Float32MultiArray, self.callback, queue_size=1)
        
        rospy.loginfo("目标跟随节点启动，正在监听目标位置...")

    def callback(self, msg):
        # msg.data 里面装的就是 [error_x, error_y]
        error_x = msg.data[0]
        error_y = msg.data[1]

        twist = Twist()
        
        # 简单的比例控制（P控制）
        # 目标在左边，error_x 是负数，小车应该左转（角速度为正）
        # 目标在右边，error_x 是正数，小车应该右转（角速度为负）
        twist.angular.z = -float(error_x) / 300.0
        
        # 如果目标的中心点偏上（距离远），error_y 是负数，小车前进（线速度为正）
        # 目标偏下（距离近），error_y 是正数，小车后退或停下
        twist.linear.x = -float(error_y) / 300.0

        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node("target_follower")
    node = TargetFollower()
    rospy.spin()
