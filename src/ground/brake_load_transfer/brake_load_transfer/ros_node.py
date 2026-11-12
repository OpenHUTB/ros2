# -*- coding: utf-8 -*-
"""ROS 2 包装节点：把准静态制动模型接到标准话题上。

订阅 ``/vehicle/brake_cmd``（``std_msgs/Float64``，0~1 的踏板开度），
以 50 Hz 积分纵向状态，发布 ``/vehicle/brake_state``
（``std_msgs/Float64MultiArray``：``[车速 m/s, 已行驶 m, 减速度 m/s², 后轴抱死?]``）。

这个节点的作用是**让模型能进 ROS 数据流**，不替代整车仿真：
横向跑偏是离线算的（见 `main.py --lateral`），因为它需要额外的侧向参数。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from .core import G, VehicleSpec, axle_brake_force, dynamic_axle_loads


class BrakeNode(Node):
    def __init__(self):
        super().__init__("brake_load_transfer_node")

        self.declare_parameter("rate", 50.0)
        self.declare_parameter("v0_kmh", 100.0)
        self.declare_parameter("mass", 1500.0)
        self.declare_parameter("payload", 0.0)
        self.declare_parameter("payload_x", 1.35)
        self.declare_parameter("brake_bias", 0.62)
        self.declare_parameter("mu", 0.90)

        gp = self.get_parameter
        self.rate = float(gp("rate").value)
        self.dt = 1.0 / self.rate
        self.spec = VehicleSpec(
            mass=float(gp("mass").value),
            payload=float(gp("payload").value),
            payload_x=float(gp("payload_x").value),
            brake_bias=float(gp("brake_bias").value),
            mu=float(gp("mu").value),
            v0=float(gp("v0_kmh").value) / 3.6,
        )

        self.v = self.spec.v0
        self.x = 0.0
        self.decel = 0.0
        self.brake = 0.0
        self.rear_locked = False
        self.stopped = False
        self._reported = False

        self.create_subscription(Float64, "/vehicle/brake_cmd", self.on_brake, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/vehicle/brake_state", 10)
        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            "brake_load_transfer 启动：总质量 %.0f kg（载重 %.0f kg @ %.2f m），"
            "β=%.2f，μ=%.2f，初速 %.0f km/h"
            % (self.spec.total_mass(), self.spec.payload, self.spec.payload_x,
               self.spec.brake_bias, self.spec.mu, self.spec.v0 * 3.6))

    # ---------------------------------------------------------------- 回调
    def on_brake(self, msg: Float64):
        self.brake = float(np.clip(msg.data, 0.0, 1.0))

    def on_timer(self):
        if not self.stopped:
            m = self.spec.total_mass()
            demand = self.brake * self.spec.mu * m * G
            fz_f, fz_r = dynamic_axle_loads(self.spec, self.decel)
            fx_f, fx_r, _, self.rear_locked = axle_brake_force(
                self.spec, fz_f, fz_r, demand)
            drag = 0.5 * self.spec.rho * self.spec.cda * self.v * self.v
            self.decel = (fx_f + fx_r + drag) / m
            self.v = max(self.v - self.decel * self.dt, 0.0)
            self.x += self.v * self.dt
            if self.v <= 1e-3:
                self.stopped = True

        if self.stopped and not self._reported:
            self._reported = True
            self.get_logger().info(
                "已停车：制动距离 %.2f m（理想 %.2f m）"
                % (self.x, self.spec.v0 ** 2 / (2 * self.spec.mu * G)))

        msg = Float64MultiArray()
        msg.data = [float(self.v), float(self.x), float(self.decel),
                    float(1.0 if self.rear_locked else 0.0)]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BrakeNode()
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
