"""Publish rendered MuJoCo observations and apply ROS velocity commands."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .landing import DT, LandingSimulation, vision_features


class LandingFeatureSource(Node):
    """Bridge MuJoCo camera/pose feedback to ROS 2 feature and command topics."""

    def __init__(self) -> None:
        super().__init__("moving_pad_landing_feature_source")
        self.simulation = LandingSimulation(width=160, height=160)
        self.simulation_time = 0.0
        self.command = np.zeros(3, dtype=np.float32)
        self.publisher = self.create_publisher(Float32MultiArray, "/landing/features", 10)
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        self.timer = self.create_timer(DT, self.publish_observation)
        self.get_logger().info("MuJoCo 特征发布与 /cmd_vel 控制已就绪")

    def on_command(self, message: Twist) -> None:
        command = np.asarray(
            [message.linear.x, message.linear.y, message.linear.z],
            dtype=np.float32,
        )
        if not np.isfinite(command).all():
            self.command = np.zeros(3, dtype=np.float32)
            return
        self.command = np.asarray(
            [np.clip(command[0], -0.55, 0.55), np.clip(command[1], -0.55, 0.55), np.clip(command[2], -0.3, 0.0)],
            dtype=np.float32,
        )

    def publish_observation(self) -> None:
        image, pad_velocity = self.simulation.render(self.simulation_time)
        features = vision_features(image, float(self.simulation.position[2]), pad_velocity)
        message = Float32MultiArray()
        message.data = [float(value) for value in features]
        self.publisher.publish(message)
        self.simulation.step(self.command, self.simulation_time)
        self.simulation_time += DT

    def destroy_node(self) -> bool:
        self.simulation.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LandingFeatureSource()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
