"""Publisher and subscriber used by the QoS compatibility demo."""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


_RELIABILITY_POLICIES = {
    "reliable": ReliabilityPolicy.RELIABLE,
    "best_effort": ReliabilityPolicy.BEST_EFFORT,
}
_DURABILITY_POLICIES = {
    "volatile": DurabilityPolicy.VOLATILE,
    "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
}


def _profile_from_parameters(node: Node) -> QoSProfile:
    reliability_name = str(
        node.get_parameter("reliability").value
    ).lower()
    durability_name = str(
        node.get_parameter("durability").value
    ).lower()
    if reliability_name not in _RELIABILITY_POLICIES:
        raise ValueError(
            "reliability must be 'reliable' or 'best_effort'"
        )
    if durability_name not in _DURABILITY_POLICIES:
        raise ValueError(
            "durability must be 'volatile' or 'transient_local'"
        )

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=int(node.get_parameter("depth").value),
        reliability=_RELIABILITY_POLICIES[reliability_name],
        durability=_DURABILITY_POLICIES[durability_name],
    )


def _declare_demo_parameters(node: Node) -> None:
    node.declare_parameter("topic", "/qos_lab/chatter")
    node.declare_parameter("reliability", "reliable")
    node.declare_parameter("durability", "volatile")
    node.declare_parameter("depth", 10)


class DemoPublisher(Node):
    """Publish numbered messages with configurable QoS."""

    def __init__(self) -> None:
        super().__init__("qos_demo_publisher")
        _declare_demo_parameters(self)
        self.declare_parameter("publish_period_sec", 1.0)
        topic = str(self.get_parameter("topic").value)
        try:
            profile = _profile_from_parameters(self)
        except ValueError as error:
            self.get_logger().fatal(str(error))
            raise
        self._publisher = self.create_publisher(String, topic, profile)
        self._count = 0
        period = float(self.get_parameter("publish_period_sec").value)
        if period <= 0.0:
            raise ValueError("publish_period_sec must be positive")
        self.create_timer(period, self._publish)
        self.get_logger().info(
            f"Demo publisher started on {topic} with reliability="
            f"{self.get_parameter('reliability').value} durability="
            f"{self.get_parameter('durability').value}"
        )

    def _publish(self) -> None:
        message = String()
        message.data = f"QoS lab message {self._count}"
        self._publisher.publish(message)
        self._count += 1


class DemoSubscriber(Node):
    """Subscribe to the demo topic with configurable QoS."""

    def __init__(self) -> None:
        super().__init__("qos_demo_subscriber")
        _declare_demo_parameters(self)
        topic = str(self.get_parameter("topic").value)
        try:
            profile = _profile_from_parameters(self)
        except ValueError as error:
            self.get_logger().fatal(str(error))
            raise
        self.create_subscription(String, topic, self._on_message, profile)
        self.get_logger().info(
            f"Demo subscriber started on {topic} with reliability="
            f"{self.get_parameter('reliability').value} durability="
            f"{self.get_parameter('durability').value}"
        )

    def _on_message(self, message: String) -> None:
        self.get_logger().info(f"Received: {message.data}")


def _run_node(node_type: type, args: Optional[List[str]]) -> None:
    rclpy.init(args=args)
    node: Optional[Node] = None
    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def publisher_main(args: Optional[List[str]] = None) -> None:
    """Run the configurable demo publisher."""

    _run_node(DemoPublisher, args)


def subscriber_main(args: Optional[List[str]] = None) -> None:
    """Run the configurable demo subscriber."""

    _run_node(DemoSubscriber, args)


if __name__ == "__main__":
    publisher_main()
