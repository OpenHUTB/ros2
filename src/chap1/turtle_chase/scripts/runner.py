#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
双海龟追逐实验 —— 逃亡者节点(turtle1)

原理: 在画布中心的圆周上放一个匀速转动的"虚拟目标点",
     逃亡者按照纯跟踪(Pure Pursuit)思想, 始终朝虚拟目标点转向,
     从而沿圆形轨迹稳定地"逃跑"; 画笔设置为绿色。
"""

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from std_srvs.srv import Empty


def normalize_angle(angle):
    """把角度差归一化到 [-pi, pi], 防止跨 +-pi 时转向跳变"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class Runner:
    def __init__(self):
        rospy.init_node('turtle_runner')

        # ---------- 可调参数(rosparam, 可用 _参数名:=值 覆盖) ----------
        self.speed = rospy.get_param('~speed', 1.2)          # 逃跑线速度 m/s
        self.radius = rospy.get_param('~radius', 2.5)        # 圆形轨迹半径 m
        self.carrot_w = rospy.get_param('~carrot_w', 0.42)   # 虚拟目标点角速度 rad/s
        self.turn_gain = rospy.get_param('~turn_gain', 5.0)  # 转向P控制增益
        self.rate = rospy.Rate(50)                           # 50Hz发布, 规避看门狗

        self.cx, self.cy = 5.544445, 5.544445   # 画布中心(turtlesim画布11x11)
        self.pose = None                        # 最新位姿, 由回调更新

        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_cb, queue_size=10)

        # 调用服务: 清空画布轨迹, 逃亡者画笔设为绿色
        rospy.wait_for_service('/clear')
        rospy.ServiceProxy('/clear', Empty)()
        rospy.wait_for_service('/turtle1/set_pen')
        rospy.ServiceProxy('/turtle1/set_pen', SetPen)(0, 255, 0, 3, 0)
        rospy.loginfo('逃亡者就绪: 沿半径 %.1f m 的圆周逃跑', self.radius)

    def pose_cb(self, msg):
        self.pose = msg

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def run(self):
        rospy.on_shutdown(lambda: self.publish_cmd(0, 0))

        # 等待第一帧位姿, 用当前位置确定虚拟目标点的初始相位
        while not rospy.is_shutdown() and self.pose is None:
            self.rate.sleep()
        p = self.pose
        phi0 = math.atan2(p.y - self.cy, p.x - self.cx)
        t0 = rospy.get_time()

        while not rospy.is_shutdown():
            p = self.pose
            # 虚拟目标点("胡萝卜")沿圆周匀速转动
            phi = phi0 + self.carrot_w * (rospy.get_time() - t0)
            tx = self.cx + self.radius * math.cos(phi)
            ty = self.cy + self.radius * math.sin(phi)
            # 期望航向 = 指向虚拟目标点的方向, 对航向误差做P控制
            err = normalize_angle(math.atan2(ty - p.y, tx - p.x) - p.theta)
            v = self.speed if abs(err) < 0.8 else 0.4 * self.speed  # 偏差大时先减速
            self.publish_cmd(v, self.turn_gain * err)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Runner().run()
    except rospy.ROSInterruptException:
        pass
