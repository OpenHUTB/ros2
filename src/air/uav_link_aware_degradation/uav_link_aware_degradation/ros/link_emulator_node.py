#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上行链路仿真节点：把 ``/pilot/cmd_vel`` 按链路损伤模型投递到 ``/uav/cmd_vel``。

这是真机部署时的占位/演示节点：真实电台/图传链路天然存在延迟、抖动与丢包，
本节点在 ROS2 进程内复现这些损伤，并向监督节点发布接收端可见的链路质量。
所有纯逻辑（延迟、Gilbert–Elliott 丢包、计划中断）在 ``core/link.py``，
本节点只做消息收发与定时器。
"""
from __future__ import annotations

import json
import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from uav_link_aware_degradation.core.link import LinkEmulator, LinkMonitor, LinkProfile


class LinkEmulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("link_emulator")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("one_way_delay", 0.12),
                ("jitter_std", 0.04),
                ("loss_good", 0.05),
                ("loss_bad", 0.30),
                ("p_good_to_bad", 0.05),
                ("p_bad_to_good", 0.30),
                ("outages", ""),
                ("seed", 0),
                ("rate_hz", 20.0),
            ],
        )
        self._p = {k: self.get_parameter(k).value for k in (
            "one_way_delay", "jitter_std", "loss_good", "loss_bad",
            "p_good_to_bad", "p_bad_to_good", "seed", "rate_hz")}
        outages_raw = self.get_parameter("outages").value or ""
        outages = tuple(
            (float(a), float(b)) for a, b in
            (x.split(":") for x in outages_raw.split(",") if x)
        )
        profile = LinkProfile(
            name="ros2", label="ROS2 上行链路",
            one_way_delay=float(self._p["one_way_delay"]),
            jitter_std=float(self._p["jitter_std"]),
            loss_good=float(self._p["loss_good"]),
            loss_bad=float(self._p["loss_bad"]),
            p_good_to_bad=float(self._p["p_good_to_bad"]),
            p_bad_to_good=float(self._p["p_bad_to_good"]),
            outages=outages,
        )
        self._emu = LinkEmulator(profile, seed=int(self._p["seed"]))
        self._mon = LinkMonitor(float(self._p["rate_hz"]), window=3.0)
        self._queue: List[Tuple[float, Twist, float]] = []
        self._t0 = self.get_clock().now()

        self.sub = self.create_subscription(
            Twist, "~/pilot_cmd", self._on_pilot, 10)
        self.pub_cmd = self.create_publisher(Twist, "~/out_cmd", 10)
        self.pub_stat = self.create_publisher(String, "~/status", 10)
        self.timer = self.create_timer(0.01, self._tick)

    def _now(self) -> float:
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _on_pilot(self, msg: Twist) -> None:
        t = self._now()
        d = self._emu.transmit(t)
        if d.delivered:
            self._queue.append((d.arrival, msg, t))

    def _tick(self) -> None:
        now = self._now()
        while self._queue and self._queue[0][0] <= now:
            _, msg, send_t = self._queue.pop(0)
            self._mon.push(send_t, now)
            self.pub_cmd.publish(msg)
        status = self._mon.update(now)
        out = String()
        out.data = json.dumps({
            "t": round(now, 3),
            "age": None if not math.isfinite(status.age) else round(status.age, 3),
            "pdr": round(status.pdr, 3),
            "jitter": None if math.isnan(status.jitter) else round(status.jitter, 3),
            "warm": status.warm,
        })
        self.pub_stat.publish(out)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = LinkEmulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
