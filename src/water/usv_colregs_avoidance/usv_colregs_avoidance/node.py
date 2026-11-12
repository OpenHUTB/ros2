#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点：订阅两船状态，发布 COLREGs 建议动作。"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .colregs import ShipState, Thresholds, decide, relative_geometry

URGENCY_CODE = {"none": 0, "warning": 1, "danger": 2}
TCPA_INF_SENTINEL = -1.0


class ColregsNode(Node):
    def __init__(self):
        super().__init__('usv_colregs_avoidance')
        self.th = Thresholds(
            cpa_safe=float(self.declare_parameter('cpa_safe_m', 500.0).value),
            cpa_warn=float(self.declare_parameter('cpa_warn_m', 300.0).value),
            tcpa_warn=float(self.declare_parameter('tcpa_warn_s', 180.0).value),
            tcpa_danger=float(self.declare_parameter('tcpa_danger_s', 60.0).value),
        )
        self.own = None
        self.other = None
        self.last_rule = None
        self.pub = self.create_publisher(Float32MultiArray, '/usv/colregs', 10)
        self.create_subscription(Float32MultiArray, '/usv/own_state',
                                 self.on_own, 10)
        self.create_subscription(Float32MultiArray, '/usv/other_state',
                                 self.on_other, 10)
        self.get_logger().info(
            'usv_colregs_avoidance ready: cpa_safe=%.0f cpa_warn=%.0f '
            'tcpa_warn=%.0f tcpa_danger=%.0f'
            % (self.th.cpa_safe, self.th.cpa_warn, self.th.tcpa_warn,
               self.th.tcpa_danger))

    def _parse(self, msg):
        data = list(msg.data)
        if len(data) < 4:
            return None
        return ShipState(data[0], data[1], math.radians(data[2]), data[3])

    def on_own(self, msg):
        self.own = self._parse(msg)
        self._evaluate()

    def on_other(self, msg):
        self.other = self._parse(msg)
        self._evaluate()

    def _evaluate(self):
        if self.own is None or self.other is None:
            return
        enc = relative_geometry(self.own, self.other, self.th)
        decision = decide(enc, self.th)
        tcpa = TCPA_INF_SENTINEL if math.isinf(enc.tcpa_s) else enc.tcpa_s
        out = Float32MultiArray()
        out.data = [float(decision.course_offset_deg),
                    float(decision.speed_scale),
                    float(URGENCY_CODE.get(decision.urgency, 0)),
                    float(enc.cpa_m), float(tcpa)]
        self.pub.publish(out)
        key = (enc.aspect, decision.rule)
        if key != self.last_rule:
            self.get_logger().info(
                '%s -> %s (%s), cpa=%.0f m tcpa=%.1f s, offset=%+.0f deg'
                % (enc.aspect, decision.action, decision.rule, enc.cpa_m,
                   tcpa, decision.course_offset_deg))
            self.last_rule = key


def main(argv=None):
    rclpy.init(args=argv)
    node = ColregsNode()
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
