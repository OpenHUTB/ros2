"""ROS 2 adapter for the AEB controller."""

from __future__ import annotations

from aeb_collision_avoidance.aeb_core import AEBConfig, AEBController


def main(args=None) -> int:
    """Run the AEB node until interrupted."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32, Float32MultiArray, String
    except ImportError as exc:  # pragma: no cover - depends on the ROS installation
        raise SystemExit(
            "ROS 2 Python packages are required: install ROS 2 and source its setup.bash"
        ) from exc

    class AEBNode(Node):
        """Publish brake commands derived from obstacle distance and ego speed."""

        def __init__(self):
            super().__init__("aeb_collision_avoidance")
            self.declare_parameter("warning_ttc", 2.5)
            self.declare_parameter("braking_ttc", 1.2)
            self.declare_parameter("stop_distance", 1.0)
            self.declare_parameter("max_brake", 8.0)
            config = AEBConfig(
                warning_ttc=float(self.get_parameter("warning_ttc").value),
                braking_ttc=float(self.get_parameter("braking_ttc").value),
                stop_distance=float(self.get_parameter("stop_distance").value),
                max_brake=float(self.get_parameter("max_brake").value),
            )
            self.controller = AEBController(config)
            self.distance = 1000.0
            self.speed = 0.0
            self.create_subscription(Float32, "/aeb/obstacle_distance", self.on_distance, 10)
            self.create_subscription(Float32, "/aeb/ego_speed", self.on_speed, 10)
            self.command_pub = self.create_publisher(Float32MultiArray, "/aeb/brake_command", 10)
            self.state_pub = self.create_publisher(String, "/aeb/state", 10)
            self.create_timer(0.05, self.tick)
            self.get_logger().info("AEB node started: waiting for /aeb/obstacle_distance and /aeb/ego_speed")

        def on_distance(self, msg):
            self.distance = max(0.0, float(msg.data))

        def on_speed(self, msg):
            self.speed = max(0.0, float(msg.data))

        def tick(self):
            decision = self.controller.decide(self.distance, self.speed)
            command = Float32MultiArray()
            command.data = [decision.brake, decision.target_speed]
            state = String()
            state.data = decision.state.value
            self.command_pub.publish(command)
            self.state_pub.publish(state)

    rclpy.init(args=args)
    node = AEBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
