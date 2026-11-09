"""Deterministic excitation commands for multi-friction data collection."""

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class VelocityCommand:
    """Planar velocity command in SI units."""

    linear: float
    angular: float


class ExcitationProfile:
    """Repeat a bounded command sequence that excites longitudinal and yaw slip."""

    def __init__(self, period_seconds: float = 12.0) -> None:
        if not math.isfinite(period_seconds) or period_seconds <= 0.0:
            raise ValueError('period_seconds must be a finite positive value')
        self._period_seconds = period_seconds

    def command_at(self, elapsed_seconds: float) -> VelocityCommand:
        """Select a command from four equal phases of the sequence."""
        if not math.isfinite(elapsed_seconds) or elapsed_seconds < 0.0:
            raise ValueError('elapsed_seconds must be finite and non-negative')
        phase = (elapsed_seconds % self._period_seconds) / self._period_seconds
        if phase < 0.25:
            return VelocityCommand(0.6, 0.0)
        if phase < 0.5:
            return VelocityCommand(0.4, 0.8)
        if phase < 0.75:
            return VelocityCommand(-0.3, 0.0)
        return VelocityCommand(0.4, -0.8)


def main(args: list[str] | None = None) -> None:
    """Publish the deterministic command profile as ROS 2 Twist messages."""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError(
            '缺少 ROS 2 Humble Python 依赖 / ROS 2 Humble Python dependencies are required'
        ) from exc

    class CommandProfileNode(Node):
        def __init__(self) -> None:
            super().__init__('slip_collection_command_profile')
            self.declare_parameter('publish_rate', 20.0)
            self.declare_parameter('profile_period', 12.0)
            rate = float(self.get_parameter('publish_rate').value)
            if not math.isfinite(rate) or rate <= 0.0:
                raise ValueError('publish_rate must be a finite positive value')
            self._profile = ExcitationProfile(
                float(self.get_parameter('profile_period').value)
            )
            self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)
            self.create_timer(1.0 / rate, self._publish_command)

        def _publish_command(self) -> None:
            elapsed = self.get_clock().now().nanoseconds / 1_000_000_000.0
            command = self._profile.command_at(elapsed)
            message = Twist()
            message.linear.x = command.linear
            message.angular.z = command.angular
            self._publisher.publish(message)

    node: Any | None = None
    rclpy.init(args=args)
    try:
        node = CommandProfileNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            stop = Twist()
            node._publisher.publish(stop)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
