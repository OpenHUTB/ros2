#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
节点2: 里程计仿真 (odom_sim)
订阅话题: /cmd_vel   (geometry_msgs/Twist)
发布话题: /odom      (nav_msgs/Odometry)
发布 TF : odom -> base_link
作用   : 代替真实机器人。订阅到速度指令后, 按差速机器人运动学积分出位姿,
         把"车体在哪儿"发布成 odom 里程计和 TF 变换。
运行(手动):  rosrun humanoid_base_sim odom_sim.py
"""
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdomSim:
    def __init__(self):
        # 订阅 cmd_vel, 一旦有速度指令就回调 _on_cmd_vel
        self.sub = rospy.Subscriber("cmd_vel", Twist, self._on_cmd_vel)
        # 发布 odom
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=50)
        # 发布 TF(用于 rviz 显示模型)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.v = 0.0      # 当前线速度 m/s
        self.w = 0.0      # 当前角速度 rad/s
        self.x = 0.0      # 位姿 x
        self.y = 0.0      # 位姿 y
        self.th = 0.0     # 航向角(rad)

        self.rate = rospy.Rate(50)     # 仿真积分频率 50 Hz
        self.last = rospy.Time.now()

    def _on_cmd_vel(self, msg):
        # 收到新速度指令, 记录下来供积分使用
        self.v = msg.linear.x
        self.w = msg.angular.z

    def _integrate(self, dt):
        # 差速运动学积分: 更新 (x, y, theta)
        if abs(self.w) > 1e-6:
            # 圆弧运动(精确)
            self.x += (self.v / self.w) * (math.sin(self.th + self.w * dt) - math.sin(self.th))
            self.y += (self.v / self.w) * (math.cos(self.th) - math.cos(self.th + self.w * dt))
            self.th += self.w * dt
        else:
            # 直线运动
            self.x += self.v * math.cos(self.th) * dt
            self.y += self.v * math.sin(self.th) * dt
        # 归一化角度到 [-pi, pi]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

    def _publish(self, now):
        # 发布 odom 消息
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.pub_odom.publish(odom)

        # 发布 TF: odom -> base_link
        ts = TransformStamped()
        ts.header.stamp = now
        ts.header.frame_id = "odom"
        ts.child_frame_id = "base_link"
        ts.transform.translation.x = self.x
        ts.transform.translation.y = self.y
        ts.transform.rotation.z = math.sin(self.th / 2.0)
        ts.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(ts)

    def run(self):
        rospy.loginfo("[odom_sim] 里程计仿真已启动, 等待 cmd_vel ...")
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last).to_sec()
            if dt > 0:
                self._integrate(dt)
                self._publish(now)
            self.last = now
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("odom_sim")
    OdomSim().run()
