#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID 8字轨迹追踪控制器
输出限幅已调小，匹配动图低速效果
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion

from mecanum_kinematics import MecanumKinematics


class PIDTracker:
    def __init__(self):
        # PID参数
        self.kp_x = rospy.get_param('~kp_x', 0.8)
        self.kp_y = rospy.get_param('~kp_y', 0.8)
        self.kp_th = rospy.get_param('~kp_th', 1.5)
        self.kd_x = rospy.get_param('~kd_x', 0.05)
        self.kd_y = rospy.get_param('~kd_y', 0.05)
        self.kd_th = rospy.get_param('~kd_th', 0.1)

        # 输出限幅（低速，匹配动图效果）
        # 0.05 m/s 对应轮子角速度约 0.05/0.08 = 0.625 rad/s
        self.max_vx = rospy.get_param('~max_vx', 0.05)
        self.max_vy = rospy.get_param('~max_vy', 0.05)
        self.max_wz = rospy.get_param('~max_wz', 0.3)

        # 运动学
        self.kin = MecanumKinematics(wheel_radius=0.08, half_length=0.18, half_width=0.15)

        # 状态
        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_th = 0.0
        self.act_x = 0.0
        self.act_y = 0.0
        self.act_th = 0.0

        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_eth = 0.0
        self.last_t = rospy.Time.now()

        # 订阅
        rospy.Subscriber('/trajectory/ref_pose', PoseStamped, self.ref_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        # 发布
        self.pub_wheels = rospy.Publisher(
            '/mecanum_group_controller/command', Float64MultiArray, queue_size=10)

        rospy.loginfo("PID轨迹追踪器启动 (低速模式)")

    def ref_cb(self, msg):
        self.ref_x = msg.pose.position.x
        self.ref_y = msg.pose.position.y
        q = msg.pose.orientation
        (_, _, self.ref_th) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def odom_cb(self, msg):
        self.act_x = msg.pose.pose.position.x
        self.act_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.act_th) = euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

    def run(self):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_t).to_sec()
            if dt <= 0:
                rate.sleep()
                continue

            # 误差
            ex = self.ref_x - self.act_x
            ey = self.ref_y - self.act_y
            eth = self.ref_th - self.act_th
            eth = np.arctan2(np.sin(eth), np.cos(eth))

            # PD控制
            vx = self.kp_x * ex + self.kd_x * (ex - self.prev_ex) / dt
            vy = self.kp_y * ey + self.kd_y * (ey - self.prev_ey) / dt
            wz = self.kp_th * eth + self.kd_th * (eth - self.prev_eth) / dt

            # 限幅（低速）
            vx = np.clip(vx, -self.max_vx, self.max_vx)
            vy = np.clip(vy, -self.max_vy, self.max_vy)
            wz = np.clip(wz, -self.max_wz, self.max_wz)

            # 逆运动学：底盘速度 -> 四轮转速
            fl, fr, rl, rr = self.kin.inverse(vx, vy, wz)

            # 发布
            msg = Float64MultiArray()
            msg.data = [fl, fr, rl, rr]
            self.pub_wheels.publish(msg)

            # 保存上次误差
            self.prev_ex = ex
            self.prev_ey = ey
            self.prev_eth = eth
            self.last_t = now

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pid_tracker')
    node = PIDTracker()
    node.run()
