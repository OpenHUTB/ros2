#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 桥接节点：神经残差控制器跑在 ROS 侧，物理跑在仿真进程里。

拓扑与底座 ``uav_sim_bridge`` 一致（一边 MuJoCo、一边 rclpy，中间 TCP），
区别是**控制律在 ROS 这一侧**：节点自己读里程计、算残差补偿、下发四桨推力。
仿真侧的 ``main.py --server`` 以 ``rotor`` 直通模式工作，不跑自己的控制器。

因此本节点**替代**底座的 ``bridge`` 节点运行（二者同时开会互相抢控制权）。

Python 3.8 兼容：本节点只依赖 numpy + rclpy，不导入 mujoco。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Optional

import numpy as np

from ..core.attitude import GRAVITY, QuadParams, quat_to_euler
from ..core.controllers import Gains, make_tracker
from ..core.remote_plant import RemotePlant
from ..core.residual import ResidualNet
from ..core.trajectories import make_trajectory


def _quat_to_ros(q):
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=float(q[1]), y=float(q[2]), z=float(q[3]), w=float(q[0]))


class ResidualControlNode:
    """把 ``Tracker`` 接到 ROS 话题与 TCP 仿真上。"""

    def __init__(self, argv=None):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Point, Twist, Vector3
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Empty, String
        from tf2_msgs.msg import TFMessage
        from uav_mujoco_sim.sim.client import SimClient, wait_for_server

        rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = Node("uav_residual_control")

        p = self.node.declare_parameters("", [
            ("host", "127.0.0.1"), ("port", 41460),
            ("controller", "geo_rls"), ("trajectory", "helix"), ("period", 8.0),
            ("net", ""), ("nominal_mass", 0.95),
            ("kp_x", -16.0), ("kp_y", -16.0), ("kp_z", -20.0),
            ("kd_x", -8.0), ("kd_y", -8.0), ("kd_z", -9.0),
            ("kr_xy", -1.5), ("kr_z", -0.6), ("kw_xy", -0.20), ("kw_z", -0.14),
            ("rls_enable", True), ("rls_lambda", 0.998), ("rls_delta", 100.0),
            ("rls_warmup", 100),
            ("control_hz", 50.0), ("steps_per_cycle", 2),
            ("sim_dt", 0.01),
        ])
        gp = {x.name: x.value for x in p}

        self.node.get_logger().info("连接仿真服务端 %s:%d …" % (gp["host"], gp["port"]))
        self.client = wait_for_server(gp["host"], gp["port"], timeout=30.0)
        head = self.client.observe(("odom",))
        self.node.get_logger().info("已连接，场景 = %s" % head.get("scene", "?"))

        self.steps = int(gp["steps_per_cycle"])
        self.sim_dt = float(gp["sim_dt"])
        self.dt = self.steps * self.sim_dt
        self.nominal_mass = float(gp["nominal_mass"])

        self.plant = RemotePlant(nominal_mass=self.nominal_mass, dt=self.dt)
        self.traj = make_trajectory(str(gp["trajectory"]), period=float(gp["period"]))
        self.controller = str(gp["controller"])

        net_path = str(gp["net"])
        self.net: Optional[ResidualNet] = None
        if net_path:
            try:
                self.net = ResidualNet.load(net_path)
                self.node.get_logger().info("已加载残差网络 %s" % net_path)
            except Exception as e:                       # noqa: BLE001
                self.node.get_logger().warn("加载残差网络失败（%s），退化为纯几何控制" % e)
        if self.net is None and self.controller.startswith("geo_r"):
            self.controller = "geo"

        g = Gains(kp=(gp["kp_x"], gp["kp_y"], gp["kp_z"]),
                  kd=(gp["kd_x"], gp["kd_y"], gp["kd_z"]),
                  kr=(gp["kr_xy"], gp["kr_xy"], gp["kr_z"]),
                  kw=(gp["kw_xy"], gp["kw_xy"], gp["kw_z"]))
        kw_rls = {}
        if not bool(gp["rls_enable"]):
            self.controller = "geo_res" if self.controller == "geo_rls" else self.controller
        else:
            kw_rls = {"lam": float(gp["rls_lambda"]), "delta": float(gp["rls_delta"]),
                      "warmup": int(gp["rls_warmup"])}
        self.tracker = make_tracker(self.controller, self.plant, self.traj,
                                    net=self.net, gains=g, **kw_rls)

        # ---- 发布
        self.pub_odom = self.node.create_publisher(Odometry, "/uav/odom", 10)
        self.pub_tf = self.node.create_publisher(TFMessage, "/tf", 10)
        self.pub_status = self.node.create_publisher(String, "/uav/status", 10)
        self.pub_ref = self.node.create_publisher(Point, "/uav/reference", 10)
        self.pub_err = self.node.create_publisher(Vector3, "/uav/tracking_error", 10)
        self.pub_res = self.node.create_publisher(Vector3, "/uav/residual", 10)
        self.pub_dist = self.node.create_publisher(Vector3, "/uav/disturbance", 10)
        self.pub_wind = self.node.create_publisher(Vector3, "/uav/wind", 10)
        # ---- 订阅
        self.node.create_subscription(String, "/uav/trajectory", self._on_traj, 10)
        self.node.create_subscription(String, "/uav/controller", self._on_ctrl, 10)
        self.node.create_subscription(Empty, "/uav/reset", self._on_reset, 10)

        self._Odom = Odometry
        self._Point, self._Vector3 = Point, Vector3
        self._String, self._TFMessage = String, TFMessage
        self.t = 0.0
        self.u = np.zeros(4)
        self._reset_to_traj()          # 必须等 tracker 建好之后再复位
        self.timer = self.node.create_timer(1.0 / float(gp["control_hz"]), self.on_control)
        self.node.get_logger().info(
            "控制器 = %s   轨迹 = %s   控制周期 = %.1f ms（仿真 %d 步/周期）"
            % (self.controller, self.traj.name, 1000 * self.dt, self.steps))

    # ---------------------------------------------------------------- 回调
    def _reset_to_traj(self) -> None:
        """把仿真复位到轨迹自身的起点（位置 + 偏航）。

        必须对齐初始偏航：四旋翼的偏航力矩受混控预算约束（悬停时上限
        0.14 N·m），偏航误差超过约 13° 就会吃光预算、把滚转/俯仰权限压掉，
        无人机翻掉且**无法自恢复**（翻过来后推力方向朝下，混控输出被限成 0）。
        从场景默认原点起飞时初始误差 2 m、偏航差 90°，实测 1.6 s 内翻机。
        """
        p0, _, _, yaw0, _ = self.traj.eval(0.0)
        self.client.reset(pos=tuple(float(x) for x in p0), yaw=float(yaw0))
        self.tracker.reset()
        self.plant.set_thrust(np.zeros(4))
        self.u = np.zeros(4)
        self.t = 0.0

    def _on_traj(self, msg) -> None:
        name = str(msg.data).strip()
        try:
            self.traj = make_trajectory(name)
            self.tracker.traj = self.traj
            self._reset_to_traj()
            self.node.get_logger().info("切换轨迹 → %s（已复位到新轨迹起点）" % name)
        except Exception as e:                           # noqa: BLE001
            self.node.get_logger().warn("未知轨迹 %r：%s" % (name, e))

    def _on_ctrl(self, msg) -> None:
        name = str(msg.data).strip()
        if name not in ("pid", "geo", "geo_res", "geo_rls"):
            self.node.get_logger().warn("未知控制器 %r" % name)
            return
        if name.startswith("geo_r") and self.net is None:
            self.node.get_logger().warn("没有残差网络，无法切到 %s" % name)
            return
        self.controller = name
        self.tracker = make_tracker(name, self.plant, self.traj, net=self.net,
                                    gains=self.tracker.g)
        self.node.get_logger().info("切换控制器 → %s" % name)

    def _on_reset(self, _msg) -> None:
        self._reset_to_traj()
        self.node.get_logger().info("仿真已复位到轨迹起点")

    # ---------------------------------------------------------------- 主回路
    def on_control(self) -> None:
        # 收到 Ctrl-C / pkill 后 rclpy 的上下文会立刻失效，而定时器可能正好
        # 卡在一次 TCP 往返里；不判 ok() 会在退出时刷一屏 RCLError 回溯。
        if not self._rclpy.ok():
            return
        try:
            f = self.client.frame(n=self.steps, rotor=self.u, want=("odom",))
        except Exception as e:                           # noqa: BLE001
            if self._rclpy.ok():
                self.node.get_logger().error("仿真通信失败：%s" % e)
            return

        od = f["odom"]
        self.plant.update(f["t"], od["pos"], od["quat"], od["vel"], od["omega"])
        self.t += self.dt
        # 顺序不能变：act → 物理步进 → observe → 下一次 act。
        # 残差网络的**输入**要用上一步下达的加速度、**标签**要用本步实测的加速度，
        # 二者之间隔着一次步进。漏掉 observe 时 RLS 与真值残差一直是 0
        # （状态里 delta_b 恒为 [0,0,0]），在线自适应完全不起作用。
        self.tracker.observe()

        u = self.tracker.act(self.t)
        self.plant.set_thrust(u)
        self.u = u

        p_ref = np.asarray(self.traj.eval(self.t)[0], float)
        err = np.asarray(self.plant.state().pos, float) - p_ref
        try:
            self._publish(f, p_ref, err)
        except Exception as e:                           # noqa: BLE001
            if self._rclpy.ok():
                raise
            self.node.get_logger().debug("退出中，忽略发布失败：%s" % e)

    # ---------------------------------------------------------------- 发布
    def _publish(self, f, p_ref, err) -> None:
        od = f["odom"]
        stamp = self.node.get_clock().now().to_msg()

        m = self._Odom()
        m.header.stamp = stamp
        m.header.frame_id = "world"
        m.child_frame_id = "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = \
            [float(v) for v in od["pos"]]
        m.pose.pose.orientation = _quat_to_ros(od["quat"])
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = \
            [float(v) for v in od["vel"]]
        m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = \
            [float(v) for v in od["omega"]]
        self.pub_odom.publish(m)

        from geometry_msgs.msg import TransformStamped
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "world"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y, \
            tf.transform.translation.z = [float(v) for v in od["pos"]]
        tf.transform.rotation = _quat_to_ros(od["quat"])
        tmsg = self._TFMessage()
        tmsg.transforms = [tf]
        self.pub_tf.publish(tmsg)

        pt = self._Point()
        pt.x, pt.y, pt.z = [float(v) for v in p_ref]
        self.pub_ref.publish(pt)

        e = self._Vector3()
        e.x, e.y, e.z = [float(v) for v in err]
        e.z = float(np.linalg.norm(err))
        self.pub_err.publish(e)

        r = self._Vector3()
        rv = self.tracker.last_residual
        r.x, r.y, r.z = [float(v) for v in rv]
        self.pub_res.publish(r)

        plant = f.get("plant") or {}
        d = self._Vector3()
        dv = plant.get("disturbance_accel", [0.0, 0.0, 0.0])
        d.x, d.y, d.z = [float(v) for v in dv]
        self.pub_dist.publish(d)

        w = self._Vector3()
        wv = plant.get("wind", [0.0, 0.0, 0.0])
        w.x, w.y, w.z = [float(v) for v in wv]
        self.pub_wind.publish(w)

        s = self._String()
        s.data = json.dumps({
            "status": "TRACKING",
            "controller": self.controller,
            "trajectory": self.traj.name,
            "pos_err": float(np.linalg.norm(err)),
            "residual_b": [round(float(v), 4) for v in self.tracker.last_residual_b],
            "delta_b": [round(float(v), 4) for v in self.tracker.last_delta],
            "thrust": [round(float(v), 3) for v in self.u],
        }, ensure_ascii=False)
        self.pub_status.publish(s)

    def spin(self) -> None:
        try:
            self._rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            # 退出前把四个桨归零，避免 Ctrl-C 后无人机还带着上一帧推力
            try:
                last = self.u.copy()
                last[:] = 0.0
                self.client.frame(n=1, rotor=last.tolist(), want=())
            except Exception:                            # noqa: BLE001
                pass
            self.client.close()
            self.node.destroy_node()
            try:
                self._rclpy.shutdown()
            except Exception:                            # noqa: BLE001
                # Ctrl-C 时 rclpy 的信号处理已经关过一次上下文，
                # 再调一次会抛 RCLError；这里静默即可
                pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="神经残差控制的 ROS2 桥接节点")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=41460)
    ap.add_argument("--controller", default="geo_rls")
    ap.add_argument("--traj", default="helix")
    ap.add_argument("--net", default="")
    ap.add_argument("--nominal-mass", type=float, default=0.95)
    a, _ = ap.parse_known_args(argv)
    try:
        node = ResidualControlNode()
    except Exception as e:                               # noqa: BLE001
        print("节点启动失败：%s" % e, file=sys.stderr)
        return 1
    if a.controller:
        node.controller = a.controller
    node.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
