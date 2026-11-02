#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_base 速度指令 -> carlair_ros_bridge 速度指令。

move_base 在 map 系下输出平面 Twist（vx, vy, yaw_rate）。无人机需要保持
当前高度，因此把 linear.z 清零后转发到桥接包的 /uav/cmd_vel；桥接包
自己做限速、机体系解释与超时悬停。

订阅：
    /cmd_vel            geometry_msgs/Twist   move_base 输出
发布：
    /uav/cmd_vel        geometry_msgs/Twist   桥接包输入（z 速度置 0）
"""
import rospy
from geometry_msgs.msg import Twist


class NavRelay(object):
    def __init__(self):
        self.timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.cb, queue_size=1)
        self.last = None
        self.last_t = 0.0

    def cb(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = 0.0          # 保持当前高度，不主动升降
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = msg.angular.z
        self.pub.publish(out)
        self.last = out
        self.last_t = rospy.get_time()

    def spin(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            # move_base 停发后超时 -> 不发指令，桥接包会自己悬停
            rate.sleep()


def main():
    rospy.init_node("nav_relay")
    NavRelay().spin()


if __name__ == "__main__":
    main()
