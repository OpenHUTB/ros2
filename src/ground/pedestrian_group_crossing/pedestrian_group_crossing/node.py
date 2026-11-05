"""ROS 2 adapter for group-crossing decisions."""

from __future__ import annotations

import json
from .group_policy import GroupCrossingPolicy


def main(args=None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray, String
    except ImportError:
        print("rclpy is not installed. Use main.py without --ros.")
        return 2

    class GroupCrossingNode(Node):
        def __init__(self) -> None:
            super().__init__("pedestrian_group_crossing")
            self.declare_parameter("group_size", 3)
            self.declare_parameter("cohesion_ratio", 0.8)
            self.declare_parameter("min_gap_seconds", 3.0)
            self.policy = GroupCrossingPolicy(
                group_size=int(self.get_parameter("group_size").value),
                cohesion_ratio=float(self.get_parameter("cohesion_ratio").value),
                min_gap_seconds=float(self.get_parameter("min_gap_seconds").value),
            )
            self.policy.validate()
            self.publisher = self.create_publisher(String, "~/decision", 10)
            self.create_subscription(
                Float32MultiArray, "~/observation", self._on_observation, 10
            )

        def _on_observation(self, message: Float32MultiArray) -> None:
            if len(message.data) < 4:
                self.get_logger().warn(
                    "observation requires [arrived, spread_m, gap_s, leader_waiting]"
                )
                return
            decision = self.policy.evaluate(
                int(message.data[0]),
                float(message.data[1]),
                float(message.data[2]),
                bool(message.data[3] >= 0.5),
            )
            output = String()
            output.data = json.dumps(decision.as_dict(), ensure_ascii=False)
            self.publisher.publish(output)

    rclpy.init(args=args)
    node = GroupCrossingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
