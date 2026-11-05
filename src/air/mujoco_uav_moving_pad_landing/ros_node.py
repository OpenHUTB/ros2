"""ROS 2 adapter: normalized image/odometry features in, Twist in world frame out."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .landing import MODEL_PATH, controller_command, load_controller


class LandingControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("moving_pad_landing_controller")
        self.model = load_controller(MODEL_PATH)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription = self.create_subscription(Float32MultiArray, "/landing/features", self.on_features, 10)
        self.get_logger().info("/landing/features -> /cmd_vel 已就绪")

    def on_features(self, message: Float32MultiArray) -> None:
        if len(message.data) != 6:
            self.get_logger().warning("输入必须包含 6 个归一化视觉/里程计特征")
            self.publisher.publish(Twist())
            return
        features = np.asarray(message.data, dtype=np.float32)
        if not np.isfinite(features).all():
            self.get_logger().warning("输入包含非有限特征，速度指令已归零")
            self.publisher.publish(Twist())
            return
        command = controller_command(features, self.model)
        output = Twist()
        output.linear.x = float(np.clip(command[0], -0.55, 0.55))
        output.linear.y = float(np.clip(command[1], -0.55, 0.55))
        output.linear.z = float(np.clip(command[2], -0.3, 0.0))
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = LandingControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
