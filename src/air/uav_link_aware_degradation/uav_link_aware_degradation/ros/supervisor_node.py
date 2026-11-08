#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分级降级监督节点：订阅链路质量与位置，按状态机决定指令来源。

远程通道：``/link_emulator/out_cmd``（受损后的遥控指令）；
机载自主：本节点按已装订航点与返航点生成速度指令。
输出统一为 ``/uav/cmd_vel``，与 ``uav_mujoco_sim`` 桥接接口一致。
"""
from __future__ import annotations

import json
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_link_aware_degradation.core.link import LinkStatus
from uav_link_aware_degradation.core.mission import Mission
from uav_link_aware_degradation.core.policies import follow
from uav_link_aware_degradation.core.supervisor import DegradationSupervisor, SupervisorParams


class SupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("degradation_supervisor")
        self.declare_parameter("mission_home", "-10.5,0.0,1.2")
        self.declare_parameter("mission_waypoints",
                               "-6.5,0.0,1.5|-1.5,0.0,1.5|3.0,0.0,1.5|6.0,0.0,1.5")
        home = np.array([float(x) for x in
                        self.get_parameter("mission_home").value.split(",")])
        wps = [np.array([float(x) for x in w.split(",")]) for w in
               self.get_parameter("mission_waypoints").value.split("|")]
        self._mission = Mission(home=home, waypoints=wps)
        self._sv = DegradationSupervisor(SupervisorParams())

        self._pos: Optional[np.ndarray] = None
        self._last_remote: Optional[Twist] = None
        self._status = LinkStatus(t=0.0, age=0.0, pdr=1.0, jitter=0.0,
                                  silent_s=0.0, warm=True)

        self.create_subscription(Odometry, "~/odom", self._on_odom, 10)
        self.create_subscription(String, "~/link_status", self._on_link, 10)
        self.create_subscription(Twist, "~/remote_cmd", self._on_remote, 10)
        self._pub = self.create_publisher(Twist, "~/cmd_vel_out", 10)
        self._pub_stat = self.create_publisher(String, "~/state", 10)
        self._t0 = self.get_clock().now()
        self.create_timer(0.02, self._tick)

    def _now(self) -> float:
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        self._pos = np.array([msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z])

    def _on_link(self, msg: String) -> None:
        d = json.loads(msg.data)
        age = d["age"] if d["age"] is not None else 1e9
        self._status = LinkStatus(t=d["t"], age=float(age), pdr=float(d["pdr"]),
                                  jitter=float(d.get("jitter") or 0.0),
                                  silent_s=float(age), warm=bool(d["warm"]))

    def _on_remote(self, msg: Twist) -> None:
        self._last_remote = msg

    def _tick(self) -> None:
        if self._pos is None:
            return
        decision = self._sv.update(self._now(), self._status, self._pos, self._mission)
        out = Twist()
        if decision.source == "remote":
            if self._last_remote is not None:
                out = self._last_remote
        else:
            v = follow(self._pos, decision.target, decision.speed_cap,
                       alt_min=0.0 if decision.mode.value == "LAND"
                       else self._sv.p.alt_min,
                       alt_max=self._sv.p.alt_max)
            out.linear.x, out.linear.y, out.linear.z = map(float, v)
        self._pub.publish(out)
        stat = String()
        stat.data = json.dumps({
            "mode": decision.mode.value,
            "source": decision.source,
            "reason": decision.reason,
            "age": round(self._status.age, 3),
            "pdr": round(self._status.pdr, 3),
        })
        self._pub_stat.publish(stat)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
