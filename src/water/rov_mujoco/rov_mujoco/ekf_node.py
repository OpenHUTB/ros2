#!/usr/bin/env python3
"""
水下多源融合导航节点（ESKF）
============================
订阅 /rov/imu_noisy、/rov/dvl、/rov/depth_noisy，输出：
    /rov/odom_fused           nav_msgs/Odometry（融合位姿 + 协方差）
    /rov/odom_deadreckoning   nav_msgs/Odometry（纯 IMU 航位推算基线）
    /rov/ekf/position_std     std_msgs/Float64MultiArray（位置标准差）
初始化：收到第一帧 IMU（含姿态）+ 深度 + DVL 后，由参数给定水平起点、
深度计确定 z、DVL 确定初始速度。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import TwistStamped

try:
    from rov_mujoco.ekf_fusion import ROVEKF, SpaceState, quat_to_rot
except ImportError:
    from ekf_fusion import ROVEKF, SpaceState, quat_to_rot


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_nav_node')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('sigma_acc', 0.05)
        self.declare_parameter('sigma_gyro', 0.01)
        self.declare_parameter('sigma_dvl', 0.01)
        self.declare_parameter('sigma_depth', 0.02)
        self.declare_parameter('sigma_att', 0.01)
        self.declare_parameter('init_delay', 0.1)
        self.sigma_dvl = float(self.get_parameter('sigma_dvl').value)
        self.sigma_depth = float(self.get_parameter('sigma_depth').value)
        self.sigma_att = float(self.get_parameter('sigma_att').value)
        self.init_delay = float(self.get_parameter('init_delay').value)

        self.ekf = None
        self.dr = None
        self.pending_depth = None
        self.pending_dvl = None
        self.pending_att = None
        self.last_stamp = None
        self.initialized_dt = False
        self.first_stamp = None

        self.sub_imu = self.create_subscription(Imu, '/rov/imu_noisy', self.on_imu, 20)
        self.sub_dvl = self.create_subscription(TwistStamped, '/rov/dvl', self.on_dvl, 20)
        self.sub_depth = self.create_subscription(Float64, '/rov/depth_noisy', self.on_depth, 20)

        self.pub_fused = self.create_publisher(Odometry, '/rov/odom_fused', 10)
        self.pub_dr = self.create_publisher(Odometry, '/rov/odom_deadreckoning', 10)
        self.pub_std = self.create_publisher(Float64MultiArray, '/rov/ekf/position_std', 10)
        self.get_logger().info('EKF 导航节点已启动，等待 IMU / DVL / 深度 ...')

    # ---------- 输入 ----------
    def on_depth(self, msg: Float64):
        self.pending_depth = float(msg.data)
        # 量测到达即更新（避免在 IMU 回调里反复使用过期缓存）
        if self.ekf is not None and self.initialized_dt:
            self.ekf.update_depth(self.pending_depth, sigma=self.sigma_depth)

    def on_dvl(self, msg: TwistStamped):
        self.pending_dvl = np.array([msg.twist.linear.x,
                                     msg.twist.linear.y,
                                     msg.twist.linear.z])
        if self.ekf is not None and self.initialized_dt:
            self.ekf.update_dvl(self.pending_dvl, sigma=self.sigma_dvl)

    def on_imu(self, msg: Imu):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        q = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        a_m = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        w_m = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        if self.ekf is None:
            # 启动瞬态：推进器从静止猛烈加速时误差会积累，
            # 等控制器稳定 init_delay 秒后再初始化滤波器。
            if self.first_stamp is None:
                self.first_stamp = stamp
            waited = (stamp[0] - self.first_stamp[0]) + 1e-9 * (stamp[1] - self.first_stamp[1])
            if waited < self.init_delay:
                return
            if self.pending_depth is None or self.pending_dvl is None:
                return
            p0 = np.array([float(self.get_parameter('init_x').value),
                           float(self.get_parameter('init_y').value),
                           -self.pending_depth])
            v0 = quat_to_rot(q) @ self.pending_dvl
            self.ekf = ROVEKF(
                p0, v0, q,
                sigma_acc=float(self.get_parameter('sigma_acc').value),
                sigma_gyro=float(self.get_parameter('sigma_gyro').value),
            )
            self.dr = SpaceState(p0, v0, q)
            self.last_stamp = stamp
            self.get_logger().info('EKF 初始化完成: p0=[%.2f %.2f %.2f]' % tuple(p0))
            return

        dt = (stamp[0] - self.last_stamp[0]) + 1e-9 * (stamp[1] - self.last_stamp[1])
        if dt <= 0:
            return
        self.last_stamp = stamp
        self.initialized_dt = True

        self.ekf.predict(a_m, w_m, dt)
        self.dr.predict(a_m, w_m, dt)
        self.ekf.update_attitude(q, sigma=self.sigma_att)

        self._publish(self.pub_fused, self.ekf.p, self.ekf.q, self.ekf.v,
                      msg.header.stamp, with_cov=True)
        self._publish(self.pub_dr, self.dr.p, self.dr.q, self.dr.v,
                      msg.header.stamp, with_cov=False)

        std = Float64MultiArray()
        std.data = [float(np.sqrt(max(v, 0.0))) for v in np.diag(self.ekf.position_cov())]
        self.pub_std.publish(std)

    def _publish(self, pub, p, q, v, stamp, with_cov):
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'world'
        od.child_frame_id = 'rov_base'
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, p)
        od.pose.pose.orientation.w, od.pose.pose.orientation.x, \
            od.pose.pose.orientation.y, od.pose.pose.orientation.z = map(float, q)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, v)
        if with_cov:
            P = self.ekf.P
            cov = np.zeros(36)
            for i in range(3):
                for j in range(3):
                    cov[i * 6 + j] = P[i, j]
            od.pose.covariance = [float(x) for x in cov]
        pub.publish(od)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
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
