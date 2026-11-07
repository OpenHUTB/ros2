#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS Noetic 节点：发布圆形轨迹的 Odometry 和 odom -> base_link TF。

轨迹数学在 ``circle_path.py``，这个文件只负责 ROS 消息封装和循环。参数：

    ~radius          圆半径 (m)，默认 1.0
    ~period          一圈秒数 (s)，默认 31.4（与旧版速度一致）
    ~rate            发布频率 (Hz)，默认 10.0
    ~frame_id        固定坐标系，默认 "odom"
    ~child_frame_id  机器人坐标系，默认 "base_link"
"""

from __future__ import annotations

import math
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry

from ros_topic_demo.circle_path import pose_at


def _quaternion(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main():
    rospy.init_node("virtual_point")

    radius = float(rospy.get_param("~radius", 1.0))
    period = float(rospy.get_param("~period", 31.4))
    rate_hz = float(rospy.get_param("~rate", 10.0))
    frame_id = str(rospy.get_param("~frame_id", "odom"))
    child_frame_id = str(rospy.get_param("~child_frame_id", "base_link"))

    if radius <= 0 or period <= 0 or rate_hz <= 0:
        rospy.logfatal("radius / period / rate 必须为正")
        return 1

    pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(rate_hz)
    omega = 2.0 * math.pi / period
    start = rospy.Time.now()

    rospy.loginfo(
        "圆形轨迹: radius=%.3f m period=%.2f s rate=%.1f Hz "
        "frame=%s child=%s",
        radius, period, rate_hz, frame_id, child_frame_id)
    rospy.loginfo("打开 rviz（Fixed Frame=%s）即可看到 Odometry 绕圈",
                  frame_id)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = (now - start).to_sec()
        phase = omega * t
        x, y, yaw = pose_at(phase, radius)
        quat = _quaternion(yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = quat
        tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        # 机体坐标下的匀速圆周运动：v = r*omega，w = omega
        odom.twist.twist.linear.x = radius * omega
        odom.twist.twist.angular.z = omega
        pub_odom.publish(odom)

        rate.sleep()

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
