#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线排队节点：订阅各机架次请求，发布当前排队序列与预计等待时间。

参数（``config/uav_vertiport_queue.yaml``）::

    uav_count: 8               # 订阅 <request_prefix>1..N/request
    request_prefix: /vertiport/uav
    queue_topic: /uav_vertiport_queue/queue
    strategy: interval         # fifo / priority / interval
    timer_period: 1.0

订阅（std_msgs/String，JSON）::

    <request_prefix><n>/request  {"uid","kind","t_request","priority","label"}

发布（std_msgs/String，JSON，判定逻辑在 core.queue.build_queue_payload）::

    <queue_topic>  {"strategy","queue":[{uid,kind,pad,t_request,
                    t_scheduled,wait}...],"summary":{...},"now":...}

本节点只做**排队编排与信息发布**，不直接控制任何一架无人机；各机按
收到的排定时刻自行执行，这条边界与参考实现一致。
"""
from __future__ import annotations

import json
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from uav_vertiport_queue.core.queue import (QueueConfig, Request,
                                            build_queue_payload)


class VertiportQueueNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_vertiport_queue")
        self.declare_parameter("uav_count", 8)
        self.declare_parameter("request_prefix", "/vertiport/uav")
        self.declare_parameter("queue_topic", "/uav_vertiport_queue/queue")
        self.declare_parameter("strategy", "interval")
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("pads_departure", 2)
        self.declare_parameter("pads_arrival", 2)
        self.declare_parameter("min_interval_departure", 60.0)
        self.declare_parameter("min_interval_arrival", 90.0)

        n = int(self.get_parameter("uav_count").value)
        prefix = str(self.get_parameter("request_prefix").value)
        self.strategy = str(self.get_parameter("strategy").value)
        self.cfg = QueueConfig(
            pads_departure=int(self.get_parameter("pads_departure").value),
            pads_arrival=int(self.get_parameter("pads_arrival").value),
            min_interval_departure=float(
                self.get_parameter("min_interval_departure").value),
            min_interval_arrival=float(
                self.get_parameter("min_interval_arrival").value))

        self.requests: Dict[Tuple[str, str], Request] = {}
        for i in range(1, n + 1):
            topic = "%s%d/request" % (prefix.rstrip("/"), i)
            self.create_subscription(
                String, topic,
                lambda msg, uid=i: self._on_request(msg), 10)

        self.pub = self.create_publisher(
            String, str(self.get_parameter("queue_topic").value), 10)
        self.create_timer(float(self.get_parameter("timer_period").value),
                          self._tick)
        self.get_logger().info(
            "排队节点已启动：%d 个请求话题，策略 %s，间隔 %.0f/%.0f s"
            % (n, self.strategy, self.cfg.min_interval_departure,
               self.cfg.min_interval_arrival))

    # ------------------------------------------------------------------ 回调
    def _on_request(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            req = Request(uid=str(data["uid"]), kind=str(data["kind"]),
                          t_request=float(data["t_request"]),
                          priority=int(data.get("priority", 0)),
                          label=str(data.get("label", "")))
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn("请求解析失败：%s（%r）" % (exc, msg.data))
            return
        self.requests[req.key] = req

    def _tick(self) -> None:
        if not self.requests:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        payload = build_queue_payload(list(self.requests.values()), self.cfg,
                                      self.strategy, now=now)
        self.pub.publish(String(
            data=json.dumps(payload, ensure_ascii=False)))
        s = payload["summary"]
        self.get_logger().info(
            "队列 %d 架次：平均等待 %.1f s，最大 %.1f s，冲突 %d"
            % (s["scheduled"], s["avg_wait"], s["max_wait"],
               s["pad_conflicts"]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VertiportQueueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
