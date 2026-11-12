# -*- coding: utf-8 -*-
"""ROS 2 节点：把 ``trajectory_msgs/JointTrajectory`` 重定时后发回去。

订阅 ``/joint_trajectory``（``trajectory_msgs/JointTrajectory``，关节名 + 关键帧），
插值求出各关节峰值速度/加速度，按 `v_max`/`a_max` 参数算最大时间缩放 α，
再发布 ``/joint_trajectory_retimed``（同一类型，时间轴已压缩），
供下游 ``joint_trajectory_controller`` 一类接口直接执行。

设计取舍：本节点**只做重定时**，不改轨迹形状、不做力矩校验——
力矩约束需要整机逆动力学，属于控制器的职责（见文档"模型边界"）。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .core import (Limits, build_trajectory, max_time_scale, retime,
                   split_duration)


class RetimeNode(Node):
    def __init__(self):
        super().__init__("joint_traj_retime_node")

        self.declare_parameter("dt", 0.001)
        self.declare_parameter("speed_scale", 1.0)      # 额外乘一个保守系数（<1 更慢）
        self.declare_parameter("default_v_max", 2.0)
        self.declare_parameter("default_a_max", 8.0)

        gp = self.get_parameter
        self.dt = float(gp("dt").value)
        self.speed_scale = float(gp("speed_scale").value)
        vmax = float(gp("default_v_max").value)
        amax = float(gp("default_a_max").value)

        self.sub = self.create_subscription(
            JointTrajectory, "/joint_trajectory", self.on_traj, 10)
        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_retimed", 10)

        self.vmax_default = vmax
        self.amax_default = amax
        self.get_logger().info(
            "joint_traj_retime 启动：dt=%.3f s，默认上限 v=%.1f rad/s、a=%.1f rad/s²，"
            "额外系数 %.2f" % (self.dt, vmax, amax, self.speed_scale))

    # ---------------------------------------------------------------- 回调
    def on_traj(self, msg: JointTrajectory):
        n_j = len(msg.joint_names)
        if n_j == 0 or len(msg.points) < 2:
            self.get_logger().warn("轨迹为空或关键帧少于 2 个，忽略")
            return

        tk = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9)
              for p in msg.points]
        qk = np.array([[float(v) for v in p.positions[:n_j]] for p in msg.points]).T

        try:
            traj = build_trajectory(tk, qk, dt=self.dt)
        except ValueError as exc:
            self.get_logger().error("无法插值：%s" % exc)
            return

        limits = Limits([self.vmax_default] * n_j, [self.amax_default] * n_j)
        try:
            r = max_time_scale(traj, limits)
        except ValueError as exc:
            self.get_logger().warn("无法重定时：%s" % exc)
            return

        alpha = r["alpha"] * max(self.speed_scale, 1e-3)
        out = JointTrajectory()
        out.header = msg.header
        out.joint_names = list(msg.joint_names)
        for p in msg.points:
            np_ = JointTrajectoryPoint()
            np_.positions = list(p.positions)
            np_.velocities = [float(v) * alpha for v in p.velocities] if p.velocities else []
            np_.accelerations = ([float(a) * alpha ** 2 for a in p.accelerations]
                                 if p.accelerations else [])
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            nt = t / alpha
            sec, nanosec = split_duration(nt)
            np_.time_from_start.sec = sec
            np_.time_from_start.nanosec = nanosec
            out.points.append(np_)

        self.pub.publish(out)
        self.get_logger().info(
            "重定时 α=%.3f：%.3f s → %.3f s（受限关节 %s，%s）"
            % (alpha, r["duration_before"], r["duration_before"] / alpha,
               msg.joint_names[r["joint"]] if r["joint"] < n_j else "?",
               r["limited_by"]))


def main(args=None):
    rclpy.init(args=args)
    node = RetimeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
