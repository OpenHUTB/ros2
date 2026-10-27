#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真机器人节点（ROS 1 / Noetic）。

发布：
    /odom          nav_msgs/Odometry     里程计（用**标称**参数积分，含系统性漂移）
    /wheel_ticks   std_msgs/Float32MultiArray  累计轮子转角 [左, 右]（编码器读数）
    /ground_truth  geometry_msgs/PoseStamped   真实位姿（相当于现场用卷尺/全站仪测量）
    TF             odom → base_link

订阅：
    /cmd_vel       geometry_msgs/Twist

为什么把真值也发出来：
    标定需要"外部测量"作为基准。真实系统里这一步是人在起点和终点拉卷尺；
    在仿真里用 /ground_truth 代替。**控制与标定算法本身不读真值以外的任何特权信息**，
    真值只用于计算误差。
"""

import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

# 支持两种运行方式：catkin 安装后（devel/install 空间）与源码目录直接运行
try:
    from diff_drive_odom_calib.model import DiffDrive, OdometryIntegrator
except ImportError:                                        # pragma: no cover
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from diff_drive_odom_calib.model import DiffDrive, OdometryIntegrator


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


class SimRobot(object):

    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.robot = DiffDrive(
            r_left=float(rospy.get_param("~r_left", 0.0505)),
            r_right=float(rospy.get_param("~r_right", 0.0495)),
            b_true=float(rospy.get_param("~b_true", 0.306)),
            r_nominal=float(rospy.get_param("~r_nominal", 0.05)),
            b_nominal=float(rospy.get_param("~b_nominal", 0.30)),
            slip_std=float(rospy.get_param("~slip_std", 0.004)),
            seed=int(rospy.get_param("~seed", 0)),
        )
        # 里程计用标称参数：这就是"未标定"的机器人
        self.odom = OdometryIntegrator(r_nominal=self.robot.r_nominal,
                                       b_nominal=self.robot.b_nominal)
        self.cmd = (0.0, 0.0)
        self.dt = 1.0 / self.rate_hz

        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.pub_ticks = rospy.Publisher("/wheel_ticks", Float32MultiArray, queue_size=10)
        self.pub_truth = rospy.Publisher("/ground_truth", PoseStamped, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster()
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=10)

        rospy.loginfo("仿真机器人启动：r_left=%.4f r_right=%.4f b_true=%.4f（标称 r=%.3f b=%.3f）",
                      self.robot.r_left, self.robot.r_right, self.robot.b_true,
                      self.robot.r_nominal, self.robot.b_nominal)

    def on_cmd(self, msg):
        self.cmd = (float(msg.linear.x), float(msg.angular.z))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            out = self.robot.step(self.cmd[0], self.cmd[1], self.dt)
            self.odom.step(out["dphi_left"], out["dphi_right"])
            stamp = rospy.Time.now()

            self.pub_odom.publish(self._make_odom(self.odom.pose, stamp))
            self.pub_ticks.publish(Float32MultiArray(
                data=[self.robot.phi_left, self.robot.phi_right]))
            self.pub_truth.publish(self._make_pose(self.robot.pose, stamp))
            self._broadcast_tf(self.odom.pose, stamp)
            rate.sleep()

    # ------------------------------------------------------------------
    def _make_pose(self, pose, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        q = yaw_to_quat(pose[2])
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    def _make_odom(self, pose, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        q = yaw_to_quat(pose[2])
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = self.cmd[0]
        msg.twist.twist.angular.z = self.cmd[1]
        # 协方差给个量级，便于 rviz / robot_localization 使用
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.005
        return msg

    def _broadcast_tf(self, pose, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        q = yaw_to_quat(pose[2])
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


def main():
    rospy.init_node("sim_robot")
    SimRobot().spin()


if __name__ == "__main__":
    main()
