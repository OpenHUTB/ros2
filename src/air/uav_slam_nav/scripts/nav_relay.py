#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_base 速度指令 -> carlair_ros_bridge 速度指令。

move_base 在 map 系下输出平面 Twist（vx, vy, yaw_rate）。无人机需要保持
当前高度，因此把 linear.z 清零后转发到桥接包的 /uav/cmd_vel；桥接包
自己做机体系解释、高度保持与超时悬停。

本节点额外做一层软件限幅，防止 move_base 规划出过大速度导致仿真不稳定。

订阅：
    /cmd_vel            geometry_msgs/Twist   move_base 输出
发布：
    /uav/cmd_vel        geometry_msgs/Twist   桥接包输入（z 速度置 0 + 限幅）

参数（launch 覆盖）：
    ~max_vx        最大线速度 x，默认 0.8 m/s
    ~max_vy        最大线速度 y，默认 0.5 m/s
    ~max_yaw       最大角速度 z，默认 0.6 rad/s
"""
import rospy
from geometry_msgs.msg import Twist


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class NavRelay(object):
    def __init__(self):
        self.max_vx = float(rospy.get_param("~max_vx", 0.8))
        self.max_vy = float(rospy.get_param("~max_vy", 0.5))
        self.max_yaw = float(rospy.get_param("~max_yaw", 0.6))
        self.pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.cb, queue_size=1)

    def cb(self, msg: Twist):
        out = Twist()
        out.linear.x = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        out.linear.y = clamp(msg.linear.y, -self.max_vy, self.max_vy)
        out.linear.z = 0.0          # 保持当前高度，不主动升降；定高由桥接包负责
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = clamp(msg.angular.z, -self.max_yaw, self.max_yaw)
        self.pub.publish(out)

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
