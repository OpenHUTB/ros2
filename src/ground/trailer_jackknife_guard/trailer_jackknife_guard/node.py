#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点：订阅铰接角/转角/请求倒车速度，发布限速后的安全速度。"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .model import TrailerParams, guard_state


class TrailerGuardNode(Node):
    def __init__(self):
        super().__init__('trailer_jackknife_guard')
        self.horizon = float(self.declare_parameter('horizon_s', 5.0).value)
        self.jackknife_deg = float(
            self.declare_parameter('jackknife_angle_deg', 75.0).value)
        self.params = TrailerParams(
            tractor_wheelbase=float(
                self.declare_parameter('tractor_wheelbase_m', 3.6).value),
            trailer_wheelbase=float(
                self.declare_parameter('trailer_wheelbase_m', 8.0).value),
            jackknife_angle=math.radians(self.jackknife_deg),
        )
        self.last_level = -1
        self.pub = self.create_publisher(Float32MultiArray, '/trailer/guard', 10)
        self.sub = self.create_subscription(
            Float32MultiArray, '/trailer/state', self.on_state, 10)
        self.get_logger().info(
            'trailer_jackknife_guard: horizon=%.1fs, jackknife=%.1f deg'
            % (self.horizon, self.jackknife_deg))

    def on_state(self, msg):
        data = list(msg.data)
        if len(data) < 3:
            self.get_logger().warning('state needs [hitch_deg, steer_deg, requested_speed]')
            return
        hitch = math.radians(data[0])
        steer = math.radians(data[1])
        requested = abs(float(data[2]))
        allowed, level, text = guard_state(hitch, steer, requested,
                                           self.params, self.horizon)
        out = Float32MultiArray()
        out.data = [float(allowed), float(level),
                    float(math.degrees(hitch)), float(math.degrees(steer))]
        self.pub.publish(out)
        if level != self.last_level:
            self.get_logger().info('risk=%d %s' % (level, text))
            self.last_level = level


def main(argv=None):
    rclpy.init(args=argv)
    node = TrailerGuardNode()
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
