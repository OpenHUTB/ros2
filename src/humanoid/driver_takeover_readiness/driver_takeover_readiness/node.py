"""ROS 2 adapter for takeover readiness."""

from __future__ import annotations

import json

from .readiness import TakeoverReadinessPolicy


def main(args=None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray, String
    except ImportError:
        print("rclpy is not installed. Use main.py without --ros.")
        return 2

    class ReadinessNode(Node):
        def __init__(self) -> None:
            super().__init__("driver_takeover_readiness")
            self.publisher = self.create_publisher(String, "~/readiness", 10)
            self.create_subscription(
                Float32MultiArray, "~/observation", self._on_observation, 10
            )

        def _on_observation(self, message: Float32MultiArray) -> None:
            if len(message.data) < 5:
                self.get_logger().warn(
                    "observation requires [gaze, hands, feet, alert_age, gaze_off]"
                )
                return
            result = TakeoverReadinessPolicy().evaluate(
                message.data[0] >= 0.5,
                message.data[1] >= 0.5,
                message.data[2] >= 0.5,
                float(message.data[3]),
                float(message.data[4]),
            )
            output = String()
            output.data = json.dumps(result.as_dict(), ensure_ascii=False)
            self.publisher.publish(output)

    rclpy.init(args=args)
    node = ReadinessNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
