#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人形机器人仿真节点 (入口 main.py)
订阅 /cmd_vel: linear.x=前进速度, angular.z=转向角速度
发布 /odom, /joint_states, tf(odom->base_footprint)
"""
import math
import os
import sys
import rospy
import roslib
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg

# 让同目录的 humanoid_model 能被 import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from humanoid_model import HumanoidModel


class HumanoidSimNode:
    def __init__(self):
        rospy.init_node("humanoid_sim_node")
        self.rate = rospy.get_param("~rate", 50.0)
        self.robot = HumanoidModel(dt=1.0 / self.rate)
        self.target_v = 0.0
        self.target_omega = 0.0

        self.odom_pub = rospy.Publisher("/odom", nav_msgs.msg.Odometry, queue_size=10)
        self.joint_pub = rospy.Publisher("/joint_states", sensor_msgs.msg.JointState, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, self._cmd_cb)
        rospy.loginfo("人形机器人仿真节点启动, 监听 /cmd_vel")

    def _cmd_cb(self, msg):
        self.target_v = msg.linear.x
        self.target_omega = msg.angular.z

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.robot.step(self.target_v, self.target_omega)
            x, y, theta, v = self.robot.state
            now = rospy.Time.now()

            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = math.sin(theta / 2.0)
            t.transform.rotation.w = math.cos(theta / 2.0)
            self.tf_broadcaster.sendTransform(t)

            odom = nav_msgs.msg.Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(theta / 2.0)
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = self.target_omega
            self.odom_pub.publish(odom)

            left_leg, right_leg = self.robot.leg_angles()
            left_arm, right_arm = self.robot.arm_angles()
            js = sensor_msgs.msg.JointState()
            js.header.stamp = now
            js.name = ["left_leg", "right_leg", "left_arm", "right_arm"]
            js.position = [left_leg, right_leg, left_arm, right_arm]
            self.joint_pub.publish(js)
            r.sleep()


if __name__ == "__main__":
    try:
        HumanoidSimNode().run()
    except rospy.ROSInterruptException:
        pass
