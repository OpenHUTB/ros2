#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_node.py — 阿克曼仿真节点 (ROS1 Noetic)

订阅:
  /cmd_vel  (geometry_msgs/Twist)
      linear.x  = 目标线速度 v (m/s)
      angular.z = 前轮转角 delta (rad)

发布:
  /odom    (nav_msgs/Odometry)
  tf: odom -> base_link

启动:
  rosrun ackermann_sim sim_node.py
  或 roslaunch ackermann_sim sim.launch
"""

import math
import os
import sys
import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg

# 让脚本直接跑时也能 import 同目录的 bike_model
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from bike_model import AckermannBike


class SimNode:
    def __init__(self):
        rospy.init_node("ackermann_sim_node", anonymous=False)

        # 从参数服务器读参数
        self.rate = rospy.get_param("~rate", 50.0)           # Hz
        wheelbase = rospy.get_param("~wheelbase", 2.5)
        max_steer = rospy.get_param("~max_steer", 0.5)
        max_speed = rospy.get_param("~max_speed", 5.0)
        max_accel = rospy.get_param("~max_accel", 2.0)

        dt = 1.0 / self.rate
        self.car = AckermannBike(wheelbase=wheelbase,
                                 max_steer=max_steer,
                                 max_speed=max_speed,
                                 max_accel=max_accel,
                                 dt=dt)

        # 收到的目标值
        self.target_v = 0.0
        self.target_delta = 0.0

        # Publisher
        self.odom_pub = rospy.Publisher("/odom", nav_msgs.msg.Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscriber
        rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, self._cmd_cb, queue_size=1)

        rospy.loginfo("Ackermann sim node started. Listening /cmd_vel (linear.x=v, angular.z=delta).")

    def _cmd_cb(self, msg):
        self.target_v = msg.linear.x
        self.target_delta = msg.angular.z

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # 用 P 控把实际速度拉到目标速度 (模拟油门/刹车)
            cur_v = self.car.v
            accel = (self.target_v - cur_v) * 2.0
            self.car.step(self.target_delta, accel)

            x, y, theta, v = self.car.state
            now = rospy.Time.now()

            # --- 发布 TF: odom -> base_link ---
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(theta / 2.0)
            t.transform.rotation.w = math.cos(theta / 2.0)
            self.tf_broadcaster.sendTransform(t)

            # --- 发布 Odometry ---
            odom = nav_msgs.msg.Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(theta / 2.0)
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = v / self.car.L * math.tan(self.target_delta)
            self.odom_pub.publish(odom)

            r.sleep()


if __name__ == "__main__":
    try:
        SimNode().run()
    except rospy.ROSInterruptException:
        pass
