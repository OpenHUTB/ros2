#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务2：激光雷达感知 + 反应式避障。

输入 /uav/lidar/scan（前向 180°），输出 /uav/cmd_vel。
策略：把扫描分成前 / 左 / 右三个扇区，取各自最近距离，
     依据距离阈值决定直行、减速转向或原地转向。
本节点不接触模拟器，可直接在录制的扫描数据上离线复现。
"""

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LidarPerception(object):
    def __init__(self):
        self.v_norm = float(rospy.get_param("~v_norm", 1.5))
        self.v_slow = float(rospy.get_param("~v_slow", 0.6))
        self.w_max = float(rospy.get_param("~w_max", 0.9))
        self.warn_dist = float(rospy.get_param("~warn_dist", 2.5))
        self.stop_dist = float(rospy.get_param("~stop_dist", 1.2))
        self.front_fov = float(rospy.get_param("~front_fov", 0.6))
        self.control = bool(rospy.get_param("~control", True))

        self.scan = None
        self.pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/uav/lidar/scan", LaserScan, self.on_scan, queue_size=1)
        rospy.loginfo("激光感知已启动（control=%s）", self.control)

    def on_scan(self, msg):
        self.scan = msg

    def sector_min(self, lo, hi):
        s = self.scan
        if s is None:
            return float("inf")
        best = float("inf")
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if lo <= a <= hi and r == r and r > 0.0:
                best = min(best, r)
        return best

    def step(self, _evt):
        cmd = Twist()
        if self.scan is None:
            self.pub.publish(cmd)
            return

        front = self.sector_min(-self.front_fov, self.front_fov)
        left = self.sector_min(0.3, 1.4)
        right = self.sector_min(-1.4, -0.3)

        if front < self.stop_dist:
            cmd.angular.z = self.w_max if left > right else -self.w_max
            cmd.linear.x = 0.0
        elif front < self.warn_dist:
            err = (left - right) / max(left + right, 1e-3)
            cmd.angular.z = max(-self.w_max, min(self.w_max, 1.2 * err))
            cmd.linear.x = self.v_slow
        else:
            cmd.linear.x = self.v_norm
            cmd.angular.z = 0.0

        if self.control:
            self.pub.publish(cmd)


def main():
    rospy.init_node("uav_lidar_perception")
    node = LidarPerception()
    rospy.Timer(rospy.Duration(0.1), node.step)
    rospy.spin()


if __name__ == "__main__":
    main()
