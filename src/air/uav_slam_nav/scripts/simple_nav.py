#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""简单避障节点：从 /scan 直接控制 /uav/cmd_vel。
前方有障碍就转向，没障碍就前进。绕过 move_base。
"""
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleNavigator:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.loginfo("simple navigator started")

    def scan_cb(self, scan):
        ranges = np.array(scan.ranges)
        finite = ranges[np.isfinite(ranges)]
        if len(finite) == 0:
            return

        # 前方 90° 范围
        n = len(ranges)
        front = ranges[n//4: 3*n//4]
        front_finite = front[np.isfinite(front)]
        if len(front_finite) == 0:
            return

        min_dist = front_finite.min()
        tw = Twist()

        if min_dist < 1.0:
            # 前方有障碍，转向
            tw.linear.x = 0.0
            tw.angular.z = 0.5
            rospy.loginfo_throttle(1, f"obstacle at {min_dist:.2f}m, turning")
        else:
            # 前进
            tw.linear.x = 0.5
            tw.angular.z = 0.0

        self.cmd_pub.publish(tw)


def main():
    rospy.init_node("simple_navigator")
    SimpleNavigator()
    rospy.spin()


if __name__ == "__main__":
    main()
