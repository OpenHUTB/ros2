#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 小海龟绘制彩色五角星 ROS Noetic
import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

# 5种颜色，5条边
COLORS = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255)]

def main():
    rospy.init_node('draw_star_node')
    # 等待画笔服务
    rospy.wait_for_service('/turtle1/set_pen')
    set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    # 速度发布器
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)
    twist = Twist()

    # 参数
    line_len = 2.0
    turn_angle_rad = 144 * math.pi / 180
    lin_speed = 1.0
    ang_speed = 1.0

    rospy.loginfo("开始绘制彩色五角星")
    for i in range(5):
        # 设置当前边颜色
        r,g,b = COLORS[i]
        set_pen(r,g,b,3,0)

        # 前进画边
        twist.linear.x = lin_speed
        twist.angular.z = 0.0
        end_time = rospy.Time.now() + rospy.Duration(line_len / lin_speed)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
        # 停下
        twist.linear.x = 0.0
        pub.publish(twist)
        rospy.sleep(0.2)

        # 转弯144度
        twist.angular.z = ang_speed
        end_time = rospy.Time.now() + rospy.Duration(turn_angle_rad / ang_speed)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
        # 停止转弯
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(0.2)
        rospy.loginfo(f"第{i+1}条边绘制完成")

    rospy.loginfo("五角星绘制完毕！")

if __name__ == '__main__':
    main()
