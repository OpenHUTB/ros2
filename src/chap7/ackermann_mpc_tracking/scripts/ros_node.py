#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MPC 跟踪节点：订阅里程计，发布 cmd_vel。

    ros2 run ackermann_mpc_tracking ros_node.py --ros-args -p v_cruise:=4.0
    ros2 topic echo /cmd_vel
    ros2 topic echo /mpc/debug      # 横向误差、参考曲率、前轮转角

话题
    订阅  /odom        nav_msgs/Odometry   车辆位姿（默认 frame: map，后轴中心）
    发布  /cmd_vel     geometry_msgs/Twist  线速度 + 角速度（差速/阿克曼底层再转换成转角）
    发布  /mpc/debug   std_msgs/Float64MultiArray  [e_y, e_psi, kappa_ref, delta]

参考路径沿用与离线演示相同的 S 弯（直线-R-L-直线），轨迹参数可通过参数覆盖。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from mpc import KinematicBicycle, LinearMPC, PathTracker, wrap_pi
import simulate as sim


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MpcTrackingNode(Node):
    def __init__(self):
        super().__init__('mpc_tracking')
        self.declare_parameter('v_cruise', 5.0)
        self.declare_parameter('horizon', 15)
        self.declare_parameter('wheelbase', 2.5)
        self.declare_parameter('delta_max', 0.6)
        self.declare_parameter('rate_max', 1.6)
        self.declare_parameter('path_radius', 6.0)
        self.declare_parameter('q_y', 10.0)
        self.declare_parameter('q_psi', 5.0)

        p = self.get_parameter
        self.v = float(p('v_cruise').value)
        self.L = float(p('wheelbase').value)
        self.delta_max = float(p('delta_max').value)
        rate_max = float(p('rate_max').value)
        horizon = int(p('horizon').value)

        self.path = sim.build_s_curve(radius=float(p('path_radius').value))
        self.model = KinematicBicycle(self.L, self.delta_max)
        self.mpc = LinearMPC(dt=0.1, horizon=horizon, wheelbase=self.L,
                             q=(float(p('q_y').value), float(p('q_psi').value)),
                             r=0.5, r_rate=40.0, delta_max=self.delta_max,
                             v_ref=self.v, rate_max=rate_max)
        self.tracker = PathTracker(self.path, s0=0.0)
        self.delta = 0.0
        self.started = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(Float64MultiArray, '/mpc/debug', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.get_logger().info(
            'MPC 跟踪节点已启动：路径长 %.2f m，预测 %d 步，v=%.2f m/s'
            % (self.path.length, horizon, self.v))

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        psi = yaw_from_quaternion(msg.pose.pose.orientation)

        info = self.tracker.update(x, y)
        e_y = info['e_y']
        e_psi = wrap_pi(psi - info['th_ref'])

        delta_ref = []
        for k in range(self.mpc.N):
            s_k = info['s'] + self.v * self.mpc.dt * (k + 1)
            delta_ref.append(self.model.delta_of_curvature(self.path.pose_at(s_k)[3]))

        d_seq, _info = self.mpc.solve([e_y, e_psi], delta_ref, delta_prev=self.delta)
        self.delta = self.mpc.delta_sequence([d_seq[0]], self.delta)[0]

        cmd = Twist()
        cmd.linear.x = self.v
        # 阿克曼底盘常用「等效角速度」接口：omega = v * tan(delta) / L
        cmd.angular.z = self.v * math.tan(self.delta) / self.L
        self.cmd_pub.publish(cmd)

        dbg = Float64MultiArray()
        dbg.data = [e_y, e_psi, info['kappa'], self.delta, info['s']]
        self.dbg_pub.publish(dbg)

        if not self.started:
            self.started = True
            self.get_logger().info('收到第一帧 /odom，开始闭环')


def main(argv=None):
    rclpy.init(args=argv)
    node = MpcTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
