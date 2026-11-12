"""ROS 2 nodes: wrench demand generation and thruster allocation."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .core import (
    DEFAULT_THRUSTERS,
    allocate_qp,
    configuration_matrix,
    effort,
    error_norm,
    limits,
)


class ThrusterSimulator(Node):
    """Publish feasible and over-scaled wrench demands."""

    def __init__(self) -> None:
        super().__init__("thruster_sim")
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("samples", 40)
        self.scale = float(self.get_parameter("scale").value)
        self.samples = max(1, int(self.get_parameter("samples").value))
        self.matrix = configuration_matrix(DEFAULT_THRUSTERS)
        self.limit = limits(DEFAULT_THRUSTERS)
        rng = np.random.default_rng(0)
        forces = rng.uniform(-1.0, 1.0, (self.samples, len(self.limit))) * self.limit
        self.wrenches = (forces @ self.matrix.T) * self.scale
        self.publisher = self.create_publisher(Wrench, "/rov/wrench_demand", 10)
        self.index = 0
        self.create_timer(0.2, self.step)
        self.get_logger().info(
            f"thruster_sim ready: {len(self.limit)} thrusters, scale {self.scale}"
        )

    def step(self) -> None:
        demand = self.wrenches[self.index % len(self.wrenches)]
        message = Wrench()
        message.force.x, message.force.y, message.force.z = (float(v) for v in demand[:3])
        message.torque.x, message.torque.y, message.torque.z = (float(v) for v in demand[3:])
        self.publisher.publish(message)
        self.index += 1


class ThrusterAllocator(Node):
    """Allocate each demand with the saturation-aware QP."""

    def __init__(self) -> None:
        super().__init__("thruster_allocator")
        self.matrix = configuration_matrix(DEFAULT_THRUSTERS)
        self.limit = limits(DEFAULT_THRUSTERS)
        self.force_pub = self.create_publisher(Float64MultiArray, "/rov/thruster_forces", 10)
        self.status_pub = self.create_publisher(String, "/rov/allocation_status", 10)
        self.create_subscription(Wrench, "/rov/wrench_demand", self.on_demand, 10)
        self.get_logger().info(
            f"thruster_allocator ready: {len(self.limit)} thrusters, "
            f"limit {self.limit[0]:.0f} N"
        )

    def on_demand(self, message: Wrench) -> None:
        demand = np.array(
            [
                message.force.x, message.force.y, message.force.z,
                message.torque.x, message.torque.y, message.torque.z,
            ]
        )
        forces = allocate_qp(self.matrix, demand, self.limit)
        output = Float64MultiArray()
        output.data = [float(value) for value in forces]
        self.force_pub.publish(output)
        status = {
            "wrench_error": error_norm(self.matrix, forces, demand),
            "effort": effort(forces),
            "at_limit": int(np.sum(np.abs(forces) >= self.limit - 1e-9)),
            "forces": forces.tolist(),
        }
        self.status_pub.publish(String(data=json.dumps(status)))
        self.get_logger().info(
            f"error {status['wrench_error']:.2f}, at limit {status['at_limit']}"
        )


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_thruster_sim(args=None) -> None:
    rclpy.init(args=args)
    _spin(ThrusterSimulator())


def main_thruster_allocator(args=None) -> None:
    rclpy.init(args=args)
    _spin(ThrusterAllocator())
