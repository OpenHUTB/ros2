#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""位姿发布节点：CarlaAir/AirSim -> /uav/odom (ENU) + TF world->base_link.

发布：
    /uav/odom        nav_msgs/Odometry    位姿与速度（ENU）
    /tf              world -> base_link

参数（可由 launch 或 rosparam 覆盖，默认值见 config/bridge.yaml）：
    sim/host, sim/airsim_port, sim/vehicle_name
    rate/odom_hz, frame/world, frame/body, topic/odom
"""
from __future__ import annotations

import os
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_client import SimClient  # noqa: E402


class OdomPublisher(object):
    def __init__(self):
        host = rospy.get_param("sim/host", "127.0.0.1")
        port = int(rospy.get_param("sim/airsim_port", 41451))
        vehicle = rospy.get_param("sim/vehicle_name", "")
        rate_hz = float(rospy.get_param("rate/odom_hz", 20.0))
        self.world_frame = rospy.get_param("frame/world", "world")
        self.body_frame = rospy.get_param("frame/body", "base_link")
        topic = rospy.get_param("topic/odom", "/uav/odom")

        self.sim = SimClient(host=host, port=port, vehicle_name=vehicle)
        self.sim.connect()
        rospy.loginfo("odom_pub: 已连接仿真器 %s:%d", host, port)

        self.pub = rospy.Publisher(topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(rate_hz)
        self.fail_count = 0

    def spin(self):
        while not rospy.is_shutdown():
            try:
                pos, quat, vel, ang = self.sim.get_state_enu()
                self.publish(pos, quat, vel, ang)
                self.fail_count = 0
            except Exception as exc:  # noqa: BLE001 - 仿真断连时不能让节点崩溃
                self.fail_count += 1
                if self.fail_count in (1, 10, 100):
                    rospy.logwarn("odom_pub: 读取位姿失败(第 %d 次): %s",
                                  self.fail_count, exc)
            self.rate.sleep()

    def publish(self, pos, quat, vel, ang):
        stamp = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        odom.twist.twist.angular.x = float(ang[0])
        odom.twist.twist.angular.y = float(ang[1])
        odom.twist.twist.angular.z = float(ang[2])
        self.pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.body_frame
        tf_msg.transform.translation.x = float(pos[0])
        tf_msg.transform.translation.y = float(pos[1])
        tf_msg.transform.translation.z = float(pos[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rospy.init_node("uav_odom_pub", anonymous=False)
    node = OdomPublisher()
    node.spin()


if __name__ == "__main__":
    main()
