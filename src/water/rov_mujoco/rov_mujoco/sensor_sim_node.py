#!/usr/bin/env python3
"""
水下传感器噪声仿真节点
======================
订阅仿真真值 /rov/odom，合成带零偏/噪声/丢包的传感器，发布：
    /rov/imu_noisy   sensor_msgs/Imu          50 Hz
    /rov/dvl         geometry_msgs/TwistStamped（机体系速度，可丢包）
    /rov/depth_noisy std_msgs/Float64（向下为正）
    /rov/ground_truth nav_msgs/Odometry（真值转发，便于评测/对比）
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

try:
    from rov_mujoco.ekf_fusion import SensorModel, quat_to_rot
except ImportError:
    from ekf_fusion import SensorModel, quat_to_rot


def quat_diff_omega(q_prev, q_now, dt):
    q_prev = np.asarray(q_prev, dtype=float)
    q_now = np.asarray(q_now, dtype=float)
    qi = np.array([q_prev[0], -q_prev[1], -q_prev[2], -q_prev[3]])
    w = qi[0] * q_now[0] - qi[1] * q_now[1] - qi[2] * q_now[2] - qi[3] * q_now[3]
    x = qi[0] * q_now[1] + qi[1] * q_now[0] + qi[2] * q_now[3] - qi[3] * q_now[2]
    y = qi[0] * q_now[2] - qi[1] * q_now[3] + qi[2] * q_now[0] + qi[3] * q_now[1]
    z = qi[0] * q_now[3] + qi[1] * q_now[2] - qi[2] * q_now[1] + qi[3] * q_now[0]
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9 or dt <= 0:
        return np.zeros(3)
    return np.array([x, y, z]) / n * (2.0 * math.atan2(n, w)) / dt


class SensorSimNode(Node):
    def __init__(self):
        super().__init__('sensor_sim_node')
        self.declare_parameter('dvl_dropout', 0.10)
        self.declare_parameter('seed', 7)
        self.declare_parameter('sigma_depth', 0.02)
        self.declare_parameter('sigma_att', 0.01)
        self.declare_parameter('bias_acc', 0.05)
        self.declare_parameter('bias_gyro', 0.003)
        self.declare_parameter('bias_dvl', 0.005)

        self.sensor = SensorModel(
            seed=int(self.get_parameter('seed').value),
            dvl_dropout=float(self.get_parameter('dvl_dropout').value),
            sigma_depth=float(self.get_parameter('sigma_depth').value),
            sigma_att=float(self.get_parameter('sigma_att').value),
            bias_acc=float(self.get_parameter('bias_acc').value),
            bias_gyro=float(self.get_parameter('bias_gyro').value),
            bias_dvl=float(self.get_parameter('bias_dvl').value),
        )

        self.pub_imu = self.create_publisher(Imu, '/rov/imu_noisy', 20)
        self.pub_dvl = self.create_publisher(TwistStamped, '/rov/dvl', 20)
        self.pub_depth = self.create_publisher(Float64, '/rov/depth_noisy', 20)
        self.pub_gt = self.create_publisher(Odometry, '/rov/ground_truth', 10)
        self.create_subscription(Odometry, '/rov/odom', self.on_odom, 20)

        self.prev = None
        self.get_logger().info('传感器噪声仿真节点已启动（IMU/DVL/深度/真值）')

    def on_odom(self, msg: Odometry):
        p = np.array([msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z])
        q = np.array([msg.pose.pose.orientation.w,
                      msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z])
        v = np.array([msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        self.pub_gt.publish(msg)

        if self.prev is None:
            self.prev = (p, v, q, msg.header.stamp)
            return

        p0, v0, q0, st0 = self.prev
        dt = (msg.header.stamp.sec - st0.sec) + 1e-9 * (msg.header.stamp.nanosec - st0.nanosec)
        if dt <= 0:
            return
        R = quat_to_rot(q)
        omega = quat_diff_omega(q0, q, dt)
        a_world = (v - v0) / dt
        self.prev = (p, v, q, msg.header.stamp)

        a_m, w_m = self.sensor.imu(R, a_world, omega, dt)
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = 'rov_imu_link'
        qn = self.sensor.attitude(q)
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = map(float, qn)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, w_m)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, a_m)
        self.pub_imu.publish(imu)

        v_body = self.sensor.dvl(R, v)
        if v_body is not None:
            dvl = TwistStamped()
            dvl.header.stamp = msg.header.stamp
            dvl.header.frame_id = 'rov_base'
            dvl.twist.linear.x, dvl.twist.linear.y, dvl.twist.linear.z = map(float, v_body)
            self.pub_dvl.publish(dvl)

        depth = Float64()
        depth.data = float(self.sensor.depth(p))
        self.pub_depth.publish(depth)


def main(args=None):
    rclpy.init(args=args)
    node = SensorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
