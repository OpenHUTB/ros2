#!/usr/bin/env python3
"""ROS 2 image-to-Twist interface with an offline-compatible core / ROS 2 图像转控制接口。"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage
except ImportError:  # ROS 2 is optional for offline replay on Windows.
    rclpy = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    import numpy as np
    from PIL import Image
    from mujoco_abandoned_park_uav_visual_steering.model import LightweightSteeringCNN
except ImportError:
    np = None
    Image = None
    LightweightSteeringCNN = None


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float
    angular_z: float
    label: str
    confidence: float


class VisualSteeringCore:
    """Pure-Python inference core shared by rclpy and replay tests."""

    def __init__(self, checkpoint: Path, steering_rate: float = 0.65) -> None:
        if Image is None or LightweightSteeringCNN is None:
            raise RuntimeError("Pillow and PyTorch are required for image inference")
        import torch

        self.torch = torch
        self.model = LightweightSteeringCNN.load_checkpoint(checkpoint)
        self.steering_rate = float(steering_rate)

    def predict(self, image: Image.Image) -> TwistCommand:
        image = image.convert("RGB").resize((self.model.image_width, self.model.image_height))
        array = np.asarray(image, dtype=np.float32) / 255.0
        tensor = self.torch.from_numpy(array.transpose(2, 0, 1)).unsqueeze(0)
        with self.torch.no_grad():
            probabilities = self.torch.softmax(self.model(tensor), dim=1)[0]
        index = int(probabilities.argmax().item())
        label = self.model.labels[index]
        confidence = float(probabilities[index].item())
        steering = {"LEFT": -1.0, "STRAIGHT": 0.0, "RIGHT": 1.0, "STOP": 0.0}[label]
        speed = 0.0 if label == "STOP" else 1.0
        return TwistCommand(speed, steering * self.steering_rate, label, confidence)


if rclpy is not None:
    class VisualSteeringNode(Node):
        """Subscribe to RGB images and publish geometry_msgs/Twist."""

        def __init__(self) -> None:
            super().__init__("visual_steering_node")
            checkpoint_value = str(self.declare_parameter("checkpoint", "").value).strip()
            checkpoint = Path(checkpoint_value) if checkpoint_value else None
            self.bridge = CvBridge() if CvBridge is not None else None
            try:
                if checkpoint is None:
                    raise FileNotFoundError("checkpoint parameter is empty; image inference is disabled")
                self.core = VisualSteeringCore(
                    checkpoint,
                    float(self.declare_parameter("steering_rate", 0.65).value),
                )
            except (FileNotFoundError, RuntimeError) as error:
                self.core = None
                self.get_logger().warning(str(error))
            image_topic = str(self.declare_parameter("image_topic", "/front_rgb/image_raw").value)
            command_topic = str(self.declare_parameter("command_topic", "/cmd_vel").value)
            self.publisher = self.create_publisher(Twist, command_topic, 10)
            self.subscription = self.create_subscription(RosImage, image_topic, self.on_image, 10)

        def on_image(self, message: RosImage) -> None:
            if self.core is None or self.bridge is None:
                return
            image = Image.fromarray(self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8"))
            command = self.core.predict(image)
            twist = Twist()
            twist.linear.x = command.linear_x
            twist.angular.z = command.angular_z
            self.publisher.publish(twist)


def ros_main() -> int:
    if rclpy is None:
        raise RuntimeError("rclpy is required for ROS 2 runtime / ROS 2 运行需要 rclpy")
    rclpy.init()
    node = VisualSteeringNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def replay_directory(checkpoint: Path, image_dir: Path) -> dict[str, object]:
    core = VisualSteeringCore(checkpoint)
    frames = sorted(image_dir.glob("*.png"))
    if not frames:
        raise ValueError("no PNG frames found / 未找到 PNG 图像")
    commands = [core.predict(Image.open(path)) for path in frames]
    return {"frames": len(commands), "labels": {label: sum(c.label == label for c in commands) for label in core.model.labels}, "commands": [command.__dict__ for command in commands]}


def main() -> int:
    if rclpy is not None and "--ros-args" in sys.argv:
        return ros_main()
    parser = argparse.ArgumentParser(description="Replay RGB images / 回放 RGB 图像")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(replay_directory(args.checkpoint.resolve(), args.images.resolve()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

