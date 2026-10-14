#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
8字（李萨如）轨迹生成器
参数已调小，匹配低速仿真
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


class TrajectoryGenerator:
    def __init__(self):
        self.A = rospy.get_param('~amplitude', 0.3)       # 振幅调小
        self.omega = rospy.get_param('~omega', 0.15)      # 角速度调小
        self.duration = rospy.get_param('~duration', 40.0) # 时长加长
        self.rate_hz = rospy.get_param('~rate', 20.0)

        self.pub_pose = rospy.Publisher('/trajectory/ref_pose', PoseStamped, queue_size=10)
        self.pub_path = rospy.Publisher('/trajectory/ref_path', Path, queue_size=10)
        self.pub_cmd = rospy.Publisher('/trajectory/ref_cmd', Twist, queue_size=10)

        self._precompute_path()
        rospy.loginfo("8字轨迹生成器启动 A=%.2f omega=%.2f", self.A, self.omega)

    def _precompute_path(self):
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        dt = 0.1
        for t in np.arange(0, self.duration, dt):
            x, y, theta, _, _, _ = self._compute(t)
            p = PoseStamped()
            p.header.frame_id = 'odom'
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.z = np.sin(theta / 2.0)
            p.pose.orientation.w = np.cos(theta / 2.0)
            self.path_msg.poses.append(p)

    def _compute(self, t):
        A, w = self.A, self.omega
        x = A * np.sin(w * t)
        y = (A / 2.0) * np.sin(2 * w * t)

        vx = A * w * np.cos(w * t)
        vy = A * w * np.cos(2 * w * t)

        theta = np.arctan2(vy, vx)

        dvx = -A * w**2 * np.sin(w * t)
        dvy = -2 * A * w**2 * np.sin(2 * w * t)
        wz = (vx * dvy - vy * dvx) / (vx**2 + vy**2 + 1e-6)

        return x, y, theta, vx, vy, wz

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now()

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - t0).to_sec()
            if t > self.duration:
                t0 = rospy.Time.now()
                t = 0.0

            x, y, theta, vx, vy, wz = self._compute(t)

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = np.sin(theta / 2.0)
            pose.pose.orientation.w = np.cos(theta / 2.0)
            self.pub_pose.publish(pose)

            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = wz
            self.pub_cmd.publish(cmd)

            self.path_msg.header.stamp = rospy.Time.now()
            self.pub_path.publish(self.path_msg)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('trajectory_generator')
    gen = TrajectoryGenerator()
    gen.run()
