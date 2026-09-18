#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
双海龟追逐实验 —— 追逐者节点(turtle2)

原理: 1) 调用 /spawn 服务动态生成第二只小海龟作为追逐者;
     2) 同时订阅两只海龟的位姿话题, 实时计算相对距离与方位;
     3) 转向P控制对准目标 + 速度P控制(抓住后保持跟随距离);
     4) 距离小于阈值判定"抓住", 变换画笔颜色并打印抓捕信息,
        随后切换为跟随模式, 稳定地跟在逃亡者身后。
"""

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen


def normalize_angle(angle):
    """把角度差归一化到 [-pi, pi], 防止跨 +-pi 时转向跳变"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class Chaser:
    def __init__(self):
        rospy.init_node('turtle_chaser')

        # ---------- 可调参数(rosparam, 可用 _参数名:=值 覆盖) ----------
        self.max_speed = rospy.get_param('~max_speed', 1.7)      # 最大线速度 m/s
        self.turn_gain = rospy.get_param('~turn_gain', 6.0)      # 转向P控制增益
        self.catch_dist = rospy.get_param('~catch_dist', 0.6)    # 抓捕判定距离 m
        self.follow_dist = rospy.get_param('~follow_dist', 0.5)  # 跟随保持距离 m
        self.rate = rospy.Rate(50)                               # 50Hz, 规避看门狗

        self.lock = threading.Lock()  # 两个位姿回调在不同线程, 加锁保护共享数据
        self.my_pose = None           # 追逐者 turtle2 的位姿
        self.target = None            # 逃亡者 turtle1 的位姿
        self.caught = False           # 是否已抓住
        self.t_start = None           # 开始追逐的时刻

        self.cmd_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle2/pose', Pose, self.my_cb, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.target_cb, queue_size=10)

        self.spawn_turtle()
        rospy.loginfo('追逐者就绪: 最大速度 %.1f m/s', self.max_speed)

    def my_cb(self, msg):
        with self.lock:
            self.my_pose = msg

    def target_cb(self, msg):
        with self.lock:
            self.target = msg

    def spawn_turtle(self):
        """调用 /spawn 服务在左下角生成追逐者, 画笔设为红色"""
        try:
            rospy.wait_for_service('/spawn', timeout=5.0)
            spawn = rospy.ServiceProxy('/spawn', Spawn)
            name = spawn(2.0, 2.0, math.pi / 4, 'turtle2').name
            rospy.loginfo('已生成追逐者: %s', name)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn('生成 turtle2 失败(可能已存在, 将直接使用): %s', e)
        try:
            rospy.wait_for_service('/turtle2/set_pen', timeout=5.0)
            rospy.ServiceProxy('/turtle2/set_pen', SetPen)(255, 0, 0, 3, 0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn('设置追逐者画笔失败: %s', e)

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def run(self):
        rospy.on_shutdown(lambda: self.publish_cmd(0, 0))

        while not rospy.is_shutdown():
            with self.lock:
                me, tgt = self.my_pose, self.target
            if me is None or tgt is None:   # 还没收到位姿, 原地等待
                self.rate.sleep()
                continue
            if self.t_start is None:
                self.t_start = rospy.get_time()

            # 相对距离与方位(极坐标)
            dx, dy = tgt.x - me.x, tgt.y - me.y
            dist = math.hypot(dx, dy)
            err = normalize_angle(math.atan2(dy, dx) - me.theta)

            # ---- 抓捕判定(只触发一次) ----
            if not self.caught and dist < self.catch_dist:
                self.caught = True
                rospy.loginfo('>>> 抓住逃亡者! 用时 %.1f s, 抓捕点 (%.2f, %.2f)',
                              rospy.get_time() - self.t_start, me.x, me.y)
                try:  # 换洋红色粗画笔, 标记抓捕后的跟随轨迹
                    rospy.ServiceProxy('/turtle2/set_pen', SetPen)(255, 0, 255, 6, 0)
                except rospy.ServiceException:
                    pass

            # ---- 速度控制: 追逐阶段全速, 跟随阶段按距离P控制 ----
            if self.caught:
                v = max(0.0, min(self.max_speed, 4.0 * (dist - self.follow_dist)))
            else:
                v = self.max_speed if abs(err) < 0.9 else 0.5
            self.publish_cmd(v, self.turn_gain * err)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Chaser().run()
    except rospy.ROSInterruptException:
        pass
