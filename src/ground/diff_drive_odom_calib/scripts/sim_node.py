#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1 仿真节点：MuJoCo 物理 + 差速车，发布里程计/编码器/IMU。

话题：
    发布  /odom          nav_msgs/Odometry        里程计（用**标称**参数积分，含漂移）
          /wheel_ticks   std_msgs/Float32MultiArray 累计轮子转角（编码器读数）
          /imu           sensor_msgs/Imu          角速度 + 线加速度
          /ground_truth  geometry_msgs/PoseStamped 真实位姿（相当于人工测量基准）
          TF             odom -> base_link
    订阅  /cmd_vel       geometry_msgs/Twist

为什么真值也发出来：
    标定必须有独立于里程计的基准。真实系统里这一步是拉卷尺；仿真里用 /ground_truth 代替。
    **控制与标定算法本身只从编码器与 IMU 拿数据**，真值只用于算误差。
"""

import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

try:
    from diff_drive_odom_calib import config
    from diff_drive_odom_calib.mujoco_env import DiffDriveEnv
except ImportError:                                          # pragma: no cover
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from diff_drive_odom_calib import config
    from diff_drive_odom_calib.mujoco_env import DiffDriveEnv


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


class SimNode(object):

    def __init__(self):
        self.scene = rospy.get_param("~scene", "") or config.resolve_scene_path(None)
        self.rate_hz = float(rospy.get_param("~rate_hz", config.CONTROL_HZ))
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        self.env = DiffDriveEnv(self.scene)
        # 里程计：用标称参数积分编码器读数（未标定的机器人）
        self.ox = self.oy = self.oth = 0.0
        self.last_ticks = (0.0, 0.0)
        self.cmd = (0.0, 0.0)

        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.pub_ticks = rospy.Publisher("/wheel_ticks", Float32MultiArray, queue_size=10)
        self.pub_imu = rospy.Publisher("/imu", Imu, queue_size=10)
        self.pub_truth = rospy.Publisher("/ground_truth", PoseStamped, queue_size=10)
        self.tf_br = TransformBroadcaster()
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=10)

        rospy.loginfo("MuJoCo 仿真启动：%s（nq=%d, nu=%d, %.0f Hz）",
                      os.path.basename(self.scene), self.env.model.nq,
                      self.env.model.nu, self.rate_hz)

    def on_cmd(self, msg):
        self.cmd = (float(msg.linear.x), float(msg.angular.z))

    # ------------------------------------------------------------------
    def _integrate_odom(self, dphi_l, dphi_r):
        r, b = config.R_NOMINAL, config.B_NOMINAL
        ds = r * 0.5 * (dphi_l + dphi_r)
        dth = r * (dphi_r - dphi_l) / b
        self.ox += ds * math.cos(self.oth + 0.5 * dth)
        self.oy += ds * math.sin(self.oth + 0.5 * dth)
        self.oth = (self.oth + dth + math.pi) % (2 * math.pi) - math.pi

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.env.set_twist(*self.cmd)
            self.env.step()
            stamp = rospy.Time.now()

            ticks = self.env.wheel_angles()
            dphi_l = ticks[0] - self.last_ticks[0]
            dphi_r = ticks[1] - self.last_ticks[1]
            self.last_ticks = ticks
            self._integrate_odom(dphi_l, dphi_r)

            # 编码器
            self.pub_ticks.publish(Float32MultiArray(data=[ticks[0], ticks[1]]))

            # IMU（真值传感器：角速度 + 线加速度）
            st = self.env.state()
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "base_link"
            # 用 MuJoCo 的 <gyro>/<accelerometer> 真实读数。
            # 早期版本把轮速反算的 st.w / st.v 塞进这两个字段：
            # linear_acceleration 的单位是 m/s²，而 st.v 是 m/s，标准消费方
            # （EKF、robot_localization）会当成加速度用，属于单位错误。
            sens = self.env.imu()
            imu.angular_velocity.x = sens["gyro_x"]
            imu.angular_velocity.y = sens["gyro_y"]
            imu.angular_velocity.z = sens["gyro_z"]
            imu.linear_acceleration.x = sens["acc_x"]
            imu.linear_acceleration.y = sens["acc_y"]
            imu.linear_acceleration.z = sens["acc_z"]
            imu.orientation.z = math.sin(0.5 * st.yaw)
            imu.orientation.w = math.cos(0.5 * st.yaw)
            # rospy 里这三个协方差字段是**元组**，不能按下标赋值（会 IndexError），
            # 必须整段赋值。这个 bug 只在虚拟机里跑才暴露出来。
            imu.orientation_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
            imu.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
            imu.linear_acceleration_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
            self.pub_imu.publish(imu)

            # 里程计
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = "odom"
            od.child_frame_id = "base_link"
            od.pose.pose.position.x = self.ox
            od.pose.pose.position.y = self.oy
            q = yaw_to_quat(self.oth)
            od.pose.pose.orientation.z = q[2]
            od.pose.pose.orientation.w = q[3]
            od.twist.twist.linear.x = st.v
            od.twist.twist.angular.z = st.w
            od.pose.covariance[0] = 0.01
            od.pose.covariance[7] = 0.01
            od.pose.covariance[35] = 0.01
            self.pub_odom.publish(od)

            # 真值
            gt = PoseStamped()
            gt.header.stamp = stamp
            gt.header.frame_id = "world"
            gt.pose.position.x = st.x
            gt.pose.position.y = st.y
            q2 = yaw_to_quat(st.yaw)
            gt.pose.orientation.z = q2[2]
            gt.pose.orientation.w = q2[3]
            self.pub_truth.publish(gt)

            if self.publish_tf:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = "odom"
                tf.child_frame_id = "base_link"
                tf.transform.translation.x = self.ox
                tf.transform.translation.y = self.oy
                tf.transform.rotation.z = q[2]
                tf.transform.rotation.w = q[3]
                self.tf_br.sendTransform(tf)

            rate.sleep()


def main():
    rospy.init_node("sim_mujoco")
    SimNode().spin()


if __name__ == "__main__":
    main()
