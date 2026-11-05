"""ROS 2 adapter: normalized image/odometry features in, Twist in world frame out."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMessage
from std_msgs.msg import Float32MultiArray

from .landing import MODEL_PATH, controller_command, load_controller


class LandingControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("moving_pad_landing_controller")
        self.declare_parameter("input_mode", "features")
        self.input_mode = str(self.get_parameter("input_mode").value)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        if self.input_mode == "image":
            from .image_controller import IMAGE_MODEL_PATH, load_image_controller

            self.image_model = load_image_controller(IMAGE_MODEL_PATH)
            self.subscription = self.create_subscription(
                ImageMessage, "/camera/image_raw", self.on_image, 10
            )
            self.get_logger().info("/camera/image_raw -> image MLP -> /cmd_vel 已就绪")
        elif self.input_mode == "features":
            self.model = load_controller(MODEL_PATH)
            self.subscription = self.create_subscription(
                Float32MultiArray, "/landing/features", self.on_features, 10
            )
            self.get_logger().info("/landing/features -> feature MLP -> /cmd_vel 已就绪")
        else:
            raise ValueError("input_mode must be 'features' or 'image'")

    def publish_command(self, command: np.ndarray) -> None:
        output = Twist()
        output.linear.x = float(np.clip(command[0], -0.55, 0.55))
        output.linear.y = float(np.clip(command[1], -0.55, 0.55))
        output.linear.z = float(np.clip(command[2], -0.3, 0.0))
        self.publisher.publish(output)

    def on_image(self, message: ImageMessage) -> None:
        if message.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warning("图像编码必须为 rgb8 或 bgr8，速度指令已归零")
            self.publisher.publish(Twist())
            return
        channels = 3
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = rows[:, : message.width * channels].reshape(message.height, message.width, channels)
        if message.encoding == "bgr8":
            image = image[:, :, ::-1].copy()
        from .image_controller import predict_image_command

        command = predict_image_command(image, self.image_model)
        self.publish_command(command)

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
        self.publish_command(command)


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
