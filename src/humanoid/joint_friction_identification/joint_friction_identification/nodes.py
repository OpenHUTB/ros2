"""ROS 2 nodes: friction-joint replay and the sweep identifier."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

from .core import identify_friction, torque_error
from .scene import JointConfig, simulate_sweeps


class FrictionSimulator(Node):
    """Publish the measured sweep torque for each commanded speed."""

    def __init__(self) -> None:
        super().__init__("friction_sim")
        self.declare_parameter("seed", 0)
        self.config = JointConfig(seed=int(self.get_parameter("seed").value))
        from .headless import TRUTH

        self.sweeps = simulate_sweeps(self.config, TRUTH)
        self.command_pub = self.create_publisher(Float64, "/joint/velocity_command", 20)
        self.state_pub = self.create_publisher(JointState, "/joint/state", 20)
        self.index = 0
        self.create_timer(0.5, self.step)
        self.get_logger().info(f"friction_sim ready: {len(self.sweeps['speeds'])} sweep points")

    def step(self) -> None:
        index = self.index % len(self.sweeps["speeds"])
        command = Float64()
        command.data = float(self.sweeps["speeds"][index])
        self.command_pub.publish(command)
        state = JointState()
        state.name = ["joint"]
        state.velocity = [float(self.sweeps["speeds"][index])]
        state.effort = [float(self.sweeps["torques"][index])]
        self.state_pub.publish(state)
        self.index += 1


class FrictionIdentifier(Node):
    """Collect sweep pairs and identify the Stribeck parameters."""

    def __init__(self) -> None:
        super().__init__("friction_identifier")
        self.declare_parameter("min_samples", 8)
        self.min_samples = max(6, int(self.get_parameter("min_samples").value))
        self.speeds: list[float] = []
        self.torques: list[float] = []
        self.create_subscription(JointState, "/joint/state", self.on_state, 30)
        self.status_pub = self.create_publisher(String, "/joint/friction", 10)
        self.done = False
        self.get_logger().info(f"friction_identifier ready: need {self.min_samples} points")

    def on_state(self, message: JointState) -> None:
        if self.done or not message.effort or not message.velocity:
            return
        speed = float(message.velocity[0])
        if abs(speed) < 1e-6:
            return
        self.speeds.append(speed)
        self.torques.append(float(message.effort[0]))
        if len(self.speeds) < self.min_samples:
            return
        try:
            model, info = identify_friction(self.speeds, self.torques)
        except ValueError as error:
            self.get_logger().warning(f"identification failed: {error}")
            return
        payload = model.to_dict()
        payload["residual_rms"] = info["residual_rms"]
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"coulomb {model.coulomb:.3f} N m, static {model.static:.3f} N m, "
            f"viscous {model.viscous:.3f}, v_s {model.stribeck_velocity:.3f}"
        )
        self.done = True


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_friction_sim(args=None) -> None:
    rclpy.init(args=args)
    _spin(FrictionSimulator())


def main_friction_identifier(args=None) -> None:
    rclpy.init(args=args)
    _spin(FrictionIdentifier())
