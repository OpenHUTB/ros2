#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四旋翼高度 PD + ESO（ADRC）抗扰的 ROS 2 节点。

数据流（传感器输入 → 观测器 → 控制输出）：

    Float64   /quad/altitude        → 订阅  高度传感器读数 y (m)
    Odometry  /quad/odom            ← 发布  被控对象真值（高度 z、垂直速度 z_dot）
    Float64   /quad/thrust          ← 发布  推力指令 u (N)
    Float64   /quad/disturbance_est ← 发布  ESO 估计的总扰动 f_hat (m/s^2)

节点内部积分四旋翼高度二阶模型 z_dd = -g + d/m + u/m，并从 /quad/altitude 读取
测量值喂给 ESO。没收到测量话题时退化为"真值 + 白噪声"，仍可独立演示。

用法：

    ros2 run quad_eso_hover ros_node.py
    ros2 launch quad_eso_hover main.launch.py
    ros2 topic echo /quad/disturbance_est     # 观察扰动估计从 -9.81 跳到 -5.81

未安装 ROS 2 时自动降级为离线仿真：

    python3 scripts/ros_node.py --offline --outdir scripts
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import eso as eso_mod  # noqa: E402

try:  # ROS 2 为可选依赖
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64

    HAS_ROS = True
except ImportError:  # pragma: no cover - 无 ROS 环境
    rclpy = None
    Node = object
    Odometry = None
    Float64 = None
    HAS_ROS = False


class QuadESONode(Node if HAS_ROS else object):
    """四旋翼高度通道：PD + 三阶线性 ESO 前馈补偿。"""

    def __init__(self, rate=200.0, dt=0.001, w0=15.0, wc=2.5, ref=1.0,
                 d_start=4.0, d_mag=4.0):
        super().__init__("quad_eso_hover")

        # ROS 参数优先：launch（w0:=... ref:=...）或 --ros-args -p 传入的值
        # 覆盖构造默认值。之前节点不声明参数，launch 改 w0/ref 完全不生效，
        # 这里显式声明并读回，构造参数仅作为默认初值（评审 #998）。
        self.declare_parameter("rate", float(rate))
        self.declare_parameter("dt", float(dt))
        self.declare_parameter("w0", float(w0))
        self.declare_parameter("wc", float(wc))
        self.declare_parameter("ref", float(ref))
        rate = float(self.get_parameter("rate").value)
        dt = float(self.get_parameter("dt").value)
        w0 = float(self.get_parameter("w0").value)
        wc = float(self.get_parameter("wc").value)
        ref = float(self.get_parameter("ref").value)

        self.plant = eso_mod.Plant(m=1.0, g=9.81)

        # 控制器带宽 wc -> PD 增益；观测器带宽 w0 -> ESO 增益
        self.kp = wc * wc
        self.kd = 2.0 * wc
        self.beta1 = 3.0 * w0
        self.beta2 = 3.0 * w0 * w0
        self.beta3 = w0 * w0 * w0

        self.dt = float(dt)
        self.period = 1.0 / float(rate)
        self.n_sub = max(1, int(round(self.period / self.dt)))

        self.ref = float(ref)
        self.d_start = float(d_start)
        self.d_mag = float(d_mag)

        # 被控对象真值与 ESO 内部状态
        self.z, self.zd = 0.0, 0.0
        self.z1, self.z2, self.z3 = 0.0, 0.0, 0.0
        self.t = 0.0
        self._meas = None
        self._meas_seen = False

        self.pub_odom = self.create_publisher(Odometry, "quad/odom", 10)
        self.pub_thrust = self.create_publisher(Float64, "quad/thrust", 10)
        self.pub_dhat = self.create_publisher(Float64, "quad/disturbance_est", 10)
        self.sub_alt = self.create_subscription(
            Float64, "quad/altitude", self._on_altitude, 10
        )
        self.timer = self.create_timer(self.period, self._tick)

        self.get_logger().info(
            "quad_eso_hover 就绪: rate=%.0fHz wc=%.2f(rad/s) w0=%.1f(rad/s) ref=%.2fm "
            "扰动 %.1fs 起 %.1fN" % (rate, wc, w0, self.ref, self.d_start, self.d_mag)
        )

    # ---------- 回调 ----------
    def _on_altitude(self, msg):
        self._meas = float(msg.data)
        if not self._meas_seen:
            self._meas_seen = True
            self.get_logger().info("收到高度传感器读数: %.3f m" % self._meas)

    def _current_disturbance(self):
        return self.d_mag if self.t >= self.d_start else 0.0

    def _tick(self):
        d = self._current_disturbance()

        # 测量：有外部高度话题就用它，否则用真值（此处不加噪声，保证可复现）
        y = self._meas if (self._meas is not None and self._meas_seen) else self.z

        # 一个控制周期内做 n_sub 个子步：控制律、被控对象、ESO 用同一步长 dt
        # 同步推进。若只在每个周期末更新一次 ESO（步长仍用 dt），观测器的
        # 等效时间尺度会比对象慢 n_sub 倍，相位滞后会把闭环推入发散。
        u = 0.0
        for _ in range(self.n_sub):
            u0 = self.kp * (self.ref - self.z1) + self.kd * (0.0 - self.z2)
            u = (u0 - self.z3) / self.plant.b0

            self.z, self.zd = eso_mod._rk4_z(self.plant, self.z, self.zd, u, d, self.dt)

            # 三阶线性 ESO（欧拉离散）
            e = self.z1 - y
            self.z1 += self.dt * (self.z2 - self.beta1 * e)
            self.z2 += self.dt * (self.z3 + self.plant.b0 * u - self.beta2 * e)
            self.z3 += self.dt * (-self.beta3 * e)

        self.t += self.period
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "quadrotor"
        odom.pose.pose.position.z = float(self.z)
        odom.twist.twist.linear.z = float(self.zd)
        self.pub_odom.publish(odom)

        m_thrust = Float64()
        m_thrust.data = float(u)
        self.pub_thrust.publish(m_thrust)

        m_d = Float64()
        m_d.data = float(self.z3)
        self.pub_dhat.publish(m_d)


def strip_ros_args(argv):
    """把 ``--ros-args`` 及其后内容留给 rclpy，argparse 只解析它前面的部分。"""
    out = []
    for a in argv:
        if a == "--ros-args":
            break
        out.append(a)
    return out


def parse_cli(argv=None):
    argv = strip_ros_args(list(sys.argv[1:] if argv is None else argv))
    ap = argparse.ArgumentParser(
        description="四旋翼高度 PD+ESO 抗扰控制（ROS 2 节点 / 离线仿真）"
    )
    ap.add_argument("--offline", action="store_true", help="不启动 ROS，仅跑离线仿真并出图")
    ap.add_argument("--outdir", default="scripts", help="离线模式的配图输出目录")
    ap.add_argument("--rate", type=float, default=200.0, help="控制频率 (Hz)")
    ap.add_argument("--dt", type=float, default=0.001, help="积分步长 (s)")
    ap.add_argument("--w0", type=float, default=15.0, help="ESO 带宽 (rad/s)")
    ap.add_argument("--ref", type=float, default=1.0, help="目标高度 (m)")
    return ap.parse_args(argv)


def run_offline(args):
    import simulate

    paths = simulate.run(args.outdir)
    print("已生成配图: %s" % (paths,))
    return 0


def main(argv=None):
    args = parse_cli(argv)

    if args.offline or not HAS_ROS:
        if not HAS_ROS and not args.offline:
            print("[提示] 未检测到 rclpy，自动切换为离线仿真模式")
        return run_offline(args)

    rclpy.init(args=list(sys.argv))
    node = QuadESONode(rate=args.rate, dt=args.dt, w0=args.w0, ref=args.ref)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
