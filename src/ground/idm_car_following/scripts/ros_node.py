#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IDM 跟车控制器的 ROS 2 节点：订阅前车里程计，输出自车加速度指令。

数据流：

    Odometry  /idm/lead_odom   → 订阅  前车状态（位置 x、速度 v）
    Odometry  /idm/ego_odom    ← 发布  自车状态（带里程计噪声）
    Float64   /idm/cmd_accel   ← 发布  IDM 解出的期望加速度 (m/s^2)

自车纵向动力学在本节点内做一阶积分（v += a·dt），因此单节点即可在 ROS 图里
构成"感知输入 → 控制律 → 控制指令"的完整回路；把 /idm/lead_odom 换成
真实前车的里程计（或 Carla / Gazebo 里发布的话题）即可接仿真器。

用法：

    ros2 run idm_car_following ros_node.py
    ros2 launch idm_car_following main.launch.py
    # 手动喂一个前车里程计（验证链路）：
    ros2 topic pub /idm/lead_odom nav_msgs/Odometry \\
      "{header: {frame_id: map}, pose: {pose: {position: {x: 30.0}}}, twist: {twist: {linear: {x: 12.0}}}}"
    ros2 topic echo /idm/cmd_accel

未安装 ROS 2 时自动降级为离线仿真：

    python3 scripts/ros_node.py --offline --outdir scripts
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import idm as idm_mod  # noqa: E402

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

VEH_LEN = 5.0  # 车身长度 (m)，用于把位置差换算成保险杠间距


class IDMNode(Node if HAS_ROS else object):
    """单车道跟驰：订阅前车里程计，发布自车加速度。"""

    def __init__(self, rate=50.0, dt=0.01, ego_v0=12.0, ego_x0=0.0, lead_x0=30.0,
                 lead_v0=15.0, a_min=-8.0):
        super().__init__("idm_car_following")

        # ROS 参数优先：launch（rate:=...）或 --ros-args -p 传入的值覆盖
        # 构造默认值。之前节点不声明参数，launch 改 rate 不生效（评审 #998）。
        self.declare_parameter("rate", float(rate))
        self.declare_parameter("dt", float(dt))
        self.declare_parameter("ego_v0", float(ego_v0))
        self.declare_parameter("ego_x0", float(ego_x0))
        self.declare_parameter("lead_x0", float(lead_x0))
        self.declare_parameter("lead_v0", float(lead_v0))
        rate = float(self.get_parameter("rate").value)
        dt = float(self.get_parameter("dt").value)
        ego_v0 = float(self.get_parameter("ego_v0").value)
        ego_x0 = float(self.get_parameter("ego_x0").value)
        lead_x0 = float(self.get_parameter("lead_x0").value)
        lead_v0 = float(self.get_parameter("lead_v0").value)

        self.model = idm_mod.IDM(v0=20.0, T=1.5, a=2.0, b=3.0, s0=2.0)
        # 输出限幅：IDM 在 gap->0 时会给出发散量级的负加速度（-10^3 m/s^2），
        # 那是数学上的趋向无穷，不是可执行指令。乘用车最大制动约 8~9 m/s^2，
        # 这里统一裁剪，保证话题上发出的都是物理可执行的加速度。
        self.a_min = float(a_min)

        self.dt = float(dt)
        self.period = 1.0 / float(rate)
        self.n_sub = max(1, int(round(self.period / self.dt)))

        # 自车状态（本节点积分）与前车状态（由话题刷新，未收到时用初值）
        self.ego_x = float(ego_x0)
        self.ego_v = float(ego_v0)
        self.lead_x = float(lead_x0)
        self.lead_v = float(lead_v0)
        self._got_lead = False

        self.pub_ego = self.create_publisher(Odometry, "idm/ego_odom", 10)
        self.pub_acc = self.create_publisher(Float64, "idm/cmd_accel", 10)
        self.sub_lead = self.create_subscription(
            Odometry, "idm/lead_odom", self._on_lead, 10
        )
        self.timer = self.create_timer(self.period, self._tick)

        self.get_logger().info(
            "idm_car_following 就绪: rate=%.0fHz v0=%.1f T=%.1f a=%.1f b=%.1f s0=%.1f"
            % (rate, self.model.v0, self.model.T, self.model.a, self.model.b, self.model.s0)
        )

    # ---------- 回调 ----------
    def _on_lead(self, msg):
        self.lead_x = float(msg.pose.pose.position.x)
        self.lead_v = float(msg.twist.twist.linear.x)
        if not self._got_lead:
            self._got_lead = True
            self.get_logger().info(
                "收到前车里程计: x=%.2f m, v=%.2f m/s" % (self.lead_x, self.lead_v)
            )

    def _tick(self):
        gap = self.lead_x - self.ego_x - VEH_LEN
        a_raw = self.model.accel(self.ego_v, self.lead_v, gap)
        # 正向已被 IDM 的 free-road 项限制在 a 以内，这里再兜住负向
        a = max(self.a_min, min(self.model.a, a_raw))

        for _ in range(self.n_sub):
            self.ego_v = max(0.0, self.ego_v + a * self.dt)
            self.ego_x += self.ego_v * self.dt

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "ego_vehicle"
        odom.pose.pose.position.x = float(self.ego_x)
        odom.twist.twist.linear.x = float(self.ego_v)
        self.pub_ego.publish(odom)

        out = Float64()
        out.data = float(a)
        self.pub_acc.publish(out)


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
        description="智能驾驶员模型（IDM）跟车（ROS 2 节点 / 离线仿真）"
    )
    ap.add_argument("--offline", action="store_true", help="不启动 ROS，仅跑离线仿真并出图")
    ap.add_argument("--outdir", default="scripts", help="离线模式的配图输出目录")
    ap.add_argument("--rate", type=float, default=50.0, help="控制频率 (Hz)")
    ap.add_argument("--dt", type=float, default=0.01, help="积分步长 (s)")
    ap.add_argument("--ego-v0", type=float, default=12.0, help="自车初速度 (m/s)")
    ap.add_argument("--lead-x0", type=float, default=30.0, help="前车初始纵向位置 (m)")
    return ap.parse_args(argv)


def run_offline(args):
    import simulate

    res = simulate.run(args.outdir)
    for k, v in res.items():
        if isinstance(v, float):
            print("%-14s %.4f" % (k, v))
        else:
            print("%-14s %s" % (k, v))
    return 0


def main(argv=None):
    args = parse_cli(argv)

    if args.offline or not HAS_ROS:
        if not HAS_ROS and not args.offline:
            print("[提示] 未检测到 rclpy，自动切换为离线仿真模式")
        return run_offline(args)

    rclpy.init(args=list(sys.argv))
    node = IDMNode(
        rate=args.rate, dt=args.dt, ego_v0=args.ego_v0, lead_x0=args.lead_x0
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
