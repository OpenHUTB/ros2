#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cart-Pole LQR 的 ROS 2 节点：仿真步进 + 闭环控制，状态与控制量都走话题。

数据流（这是 ROS 图里真实存在的一条链路）：

    JointState  /cartpole/joint_states  ← 发布  小车位置 x、摆角 theta（及速度）
    Float64     /cartpole/force         ← 发布  LQR 解出的小车推力 u (N)
    Float64     /cartpole/force_cmd     → 订阅  外部接管推力（1 s 内有效，过期回落到 LQR）

用法：

    ros2 run cartpole_lqr ros_node.py                 # 内部 LQR 闭环
    ros2 run cartpole_lqr ros_node.py --ros-args -p use_lqr:=false   # 无控对照
    ros2 launch cartpole_lqr main.launch.py
    ros2 topic echo /cartpole/joint_states            # 观察状态
    ros2 topic pub /cartpole/force_cmd std_msgs/Float64 "{data: 5.0}" # 手动推一把

未安装 ROS 2（无法 import rclpy）时自动降级为离线仿真：

    python3 scripts/ros_node.py --offline --outdir scripts
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cartpole as cp  # noqa: E402

try:  # ROS 2 为可选依赖，缺失时仍可跑离线仿真
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64

    HAS_ROS = True
except ImportError:  # pragma: no cover - 无 ROS 环境
    rclpy = None
    Node = object
    JointState = None
    Float64 = None
    HAS_ROS = False


class CartPoleNode(Node if HAS_ROS else object):
    """倒立摆仿真 + LQR 控制器，合并在一个节点内（便于单命令演示）。"""

    def __init__(self, theta0=0.1, rate=100.0, dt=0.0025, use_lqr=True):
        super().__init__("cartpole_lqr")
        # ROS 参数优先：launch（use_lqr:=...）或 --ros-args -p 传入的值
        # 覆盖构造默认值（use_lqr 原本就声明；theta0/rate/dt 一并声明，
        # 与另两个包保持同一参数模式，评审 #998）。
        self.declare_parameter("use_lqr", bool(use_lqr))
        self.declare_parameter("theta0", float(theta0))
        self.declare_parameter("rate", float(rate))
        self.declare_parameter("dt", float(dt))
        use_lqr = bool(self.get_parameter("use_lqr").value)
        theta0 = float(self.get_parameter("theta0").value)
        rate = float(self.get_parameter("rate").value)
        dt = float(self.get_parameter("dt").value)
        self.use_lqr = use_lqr

        # 线性化与 LQR 设计（与离线仿真完全同一套代码）
        self.model, self.K, _, _, _ = cp.default_problem(0.01)
        self.state = [0.0, 0.0, float(theta0), 0.0]

        self.period = 1.0 / float(rate)
        self.dt = float(dt)
        # 每个控制周期内做 n_sub 次定步长积分，保证积分精度与 rate 解耦
        self.n_sub = max(1, int(round(self.period / self.dt)))

        self._ext_cmd = None
        self._ext_until = 0.0

        self.pub_state = self.create_publisher(JointState, "cartpole/joint_states", 10)
        self.pub_force = self.create_publisher(Float64, "cartpole/force", 10)
        self.sub_cmd = self.create_subscription(
            Float64, "cartpole/force_cmd", self._on_force_cmd, 10
        )
        self.timer = self.create_timer(self.period, self._tick)

        k = self.K[0]
        self.get_logger().info(
            "cartpole_lqr 就绪: rate=%.0fHz dt=%.4fs use_lqr=%s K=[%.3f, %.3f, %.3f, %.3f]"
            % (rate, self.dt, self.use_lqr, k[0], k[1], k[2], k[3])
        )

    # ---------- 内部工具 ----------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lqr_force(self):
        """u = -K x（K 为 1x4 行向量）。"""
        s = self.state
        k = self.K[0]
        return -(k[0] * s[0] + k[1] * s[1] + k[2] * s[2] + k[3] * s[3])

    # ---------- 回调 ----------
    def _on_force_cmd(self, msg):
        self._ext_cmd = float(msg.data)
        self._ext_until = self._now() + 1.0
        self.get_logger().info("外部接管推力: %.3f N（1s 内有效）" % self._ext_cmd)

    def _tick(self):
        if self._ext_cmd is not None and self._now() <= self._ext_until:
            u = float(self._ext_cmd)
        elif self.use_lqr:
            u = self._lqr_force()
        else:
            u = 0.0

        for _ in range(self.n_sub):
            self.state = cp.rk4(self.model, self.state, u, self.dt)

        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = ["cart_position", "pole_angle"]
        js.position = [float(self.state[0]), float(self.state[2])]
        js.velocity = [float(self.state[1]), float(self.state[3])]
        self.pub_state.publish(js)

        out = Float64()
        out.data = float(u)
        self.pub_force.publish(out)


def strip_ros_args(argv):
    """把 ``--ros-args`` 及其后内容留给 rclpy，argparse 只解析它前面的部分。

    这样 ``ros2 run ... --ros-args -p use_lqr:=false`` 不会让 argparse 报
    "unrecognized arguments"。
    """
    out = []
    for a in argv:
        if a == "--ros-args":
            break
        out.append(a)
    return out


def parse_cli(argv=None):
    argv = strip_ros_args(list(sys.argv[1:] if argv is None else argv))
    ap = argparse.ArgumentParser(
        description="倒立摆 Cart-Pole 的 LQR 平衡控制（ROS 2 节点 / 离线仿真）"
    )
    ap.add_argument("--offline", action="store_true", help="不启动 ROS，仅跑离线仿真并出图")
    ap.add_argument("--outdir", default="scripts", help="离线模式的配图输出目录")
    ap.add_argument("--theta0", type=float, default=0.1, help="初始摆角 (rad)")
    ap.add_argument("--rate", type=float, default=100.0, help="控制频率 (Hz)")
    ap.add_argument("--dt", type=float, default=0.0025, help="积分步长 (s)")
    ap.add_argument("--no-lqr", action="store_true", help="禁用 LQR（自由倒下对照组）")
    return ap.parse_args(argv)


def run_offline(args):
    import simulate

    res = simulate.run(args.outdir, theta0=args.theta0)
    print("LQR 增益 K = [%.3f, %.3f, %.3f, %.3f]" % tuple(res["K"]))
    print(
        "LQR   : 末端摆角 %.5f rad, 末端位置 %.5f m, 峰值控制力 %.2f N"
        % (res["theta_end_lqr"], res["pos_end_lqr"], res["max_force"])
    )
    print("无控  : 末端摆角 %.3f rad（已倒下）" % res["theta_end_passive"])
    print("已生成: %s  cartpole_angle.png  cartpole_state.png" % args.outdir)
    return 0


def main(argv=None):
    args = parse_cli(argv)

    if args.offline or not HAS_ROS:
        if not HAS_ROS and not args.offline:
            print("[提示] 未检测到 rclpy，自动切换为离线仿真模式")
        return run_offline(args)

    rclpy.init(args=list(sys.argv))
    node = CartPoleNode(
        theta0=args.theta0, rate=args.rate, dt=args.dt, use_lqr=not args.no_lqr
    )
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
