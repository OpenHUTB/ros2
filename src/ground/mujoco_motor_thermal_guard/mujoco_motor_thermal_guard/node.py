"""ROS 2 adapter for standard temperature and velocity messages."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32

from mujoco_motor_thermal_guard.guard import MotorTemperatureGuard, update_guard_sample


class MotorThermalGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_thermal_guard")
        self.declare_parameter("limit_c", 65.0)
        self.declare_parameter("release_c", 60.0)
        self.declare_parameter("horizon_s", 5.0)
        self.declare_parameter("temperature_timeout_s", 0.5)
        self.declare_parameter("temperature_topic", "/motor/temperature")
        self.declare_parameter("command_topic", "/cmd_vel")
        self.guard = MotorTemperatureGuard(
            self.get_parameter("limit_c").value,
            self.get_parameter("release_c").value,
            self.get_parameter("horizon_s").value,
        )
        self.speed_scale = 1.0
        self.last_temperature_received_s: float | None = None
        self.temperature_sub = self.create_subscription(
            Temperature, self.get_parameter("temperature_topic").value, self._temperature, 10
        )
        self.command_sub = self.create_subscription(
            Twist, self.get_parameter("command_topic").value, self._command, 10
        )
        self.command_pub = self.create_publisher(Twist, "/thermal_guard/cmd_vel", 10)
        self.prediction_pub = self.create_publisher(Float32, "/thermal_guard/predicted_temperature", 10)
        self.scale_pub = self.create_publisher(Float32, "/thermal_guard/speed_scale", 10)

    def _temperature(self, message: Temperature) -> None:
        stamp = message.header.stamp
        sample_time = stamp.sec + stamp.nanosec * 1e-9
        if sample_time <= 0:
            sample_time = self.get_clock().now().nanoseconds * 1e-9
        decision, error = update_guard_sample(self.guard, sample_time, float(message.temperature))
        if decision is None:
            self.get_logger().warning(f"Discarding invalid temperature sample: {error}")
            return
        self.speed_scale = decision.speed_scale
        self.last_temperature_received_s = self.get_clock().now().nanoseconds * 1e-9
        prediction_message = Float32()
        prediction_message.data = decision.predicted_temperature_c
        self.prediction_pub.publish(prediction_message)
        scale_message = Float32()
        scale_message.data = decision.speed_scale
        self.scale_pub.publish(scale_message)

    def _command(self, message: Twist) -> None:
        outgoing = Twist()
        now = self.get_clock().now().nanoseconds * 1e-9
        timeout = float(self.get_parameter("temperature_timeout_s").value)
        age = None if self.last_temperature_received_s is None else now - self.last_temperature_received_s
        fresh = age is not None and 0.0 <= age <= timeout
        scale = self.speed_scale if fresh else 0.0
        for vector in ("linear", "angular"):
            for axis in ("x", "y", "z"):
                setattr(getattr(outgoing, vector), axis, getattr(getattr(message, vector), axis) * scale)
        self.command_pub.publish(outgoing)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotorThermalGuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
