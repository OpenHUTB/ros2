"""ROS 2 adapter for the crossing policy.

Input topic:  /pedestrian_crossing_policy/observation  (Float32MultiArray)
              data = [distance_m, closing_speed_mps]
Output topic: /pedestrian_crossing_policy/wait             (Bool)

The node is optional. The pure policy and CLI do not import rclpy.
"""

from __future__ import annotations

from .policy import CrossingPolicy


def main(args=None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float32MultiArray
    except ImportError:
        print("rclpy is not installed. Use the CLI without --ros or install ROS 2.")
        return 2

    class CrossingPolicyNode(Node):
        def __init__(self) -> None:
            super().__init__("pedestrian_crossing_policy")
            self.declare_parameter("min_time_to_collision", 3.0)
            self.declare_parameter("max_vehicle_distance", 12.0)
            self.declare_parameter("min_vehicle_distance", 1.5)
            self.policy = CrossingPolicy(
                min_time_to_collision=float(
                    self.get_parameter("min_time_to_collision").value
                ),
                max_vehicle_distance=float(
                    self.get_parameter("max_vehicle_distance").value
                ),
                min_vehicle_distance=float(
                    self.get_parameter("min_vehicle_distance").value
                ),
            )
            self.policy.validate()
            self.publisher = self.create_publisher(
                Bool, "/pedestrian_crossing_policy/wait", 10
            )
            self.create_subscription(
                Float32MultiArray,
                "/pedestrian_crossing_policy/observation",
                self._on_observation,
                10,
            )

        def _on_observation(self, message: Float32MultiArray) -> None:
            if len(message.data) < 2:
                self.get_logger().warn("observation requires [distance, closing_speed]")
                return
            distance = float(message.data[0])
            closing_speed = float(message.data[1])
            result = Bool()
            result.data = self.policy.should_wait(distance, closing_speed)
            self.publisher.publish(result)
            self.get_logger().info(
                "distance=%.2f closing=%.2f wait=%s"
                % (distance, closing_speed, result.data)
            )

    rclpy.init(args=args)
    node = CrossingPolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
