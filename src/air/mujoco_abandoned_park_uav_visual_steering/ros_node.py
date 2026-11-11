"""ROS 2 image-to-Twist interface and offline RGB replay / 图像转控制与离线回放。"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

try:
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage
except ImportError:  # Offline replay does not require ROS / 离线回放无需 ROS。
    rclpy = None


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float
    angular_z: float
    label: str
    confidence: float


class VisualSteeringCore:
    """Shared inference core with positive yaw meaning left / 正偏航表示左转。"""

    def __init__(self, checkpoint: Path, steering_rate: float = 0.65) -> None:
        import torch
        from mujoco_abandoned_park_uav_visual_steering.model import LightweightSteeringCNN

        if not math.isfinite(steering_rate) or steering_rate < 0.0:
            raise ValueError("steering_rate must be finite and nonnegative / 转向率须有限且非负")
        self.torch = torch
        self.model = LightweightSteeringCNN.load_checkpoint(checkpoint)
        self.steering_rate = float(steering_rate)

    def predict(self, image: Image.Image) -> TwistCommand:
        image = image.convert("RGB").resize((self.model.image_width, self.model.image_height))
        array = np.asarray(image, dtype=np.float32) / 255.0
        tensor = self.torch.from_numpy(array.transpose(2, 0, 1)).unsqueeze(0)
        with self.torch.no_grad():
            probabilities = self.torch.softmax(self.model(tensor), dim=1)[0]
        if not bool(self.torch.isfinite(probabilities).all()):
            raise ValueError("non-finite inference result / 推理结果包含非有限值")
        index = int(probabilities.argmax().item())
        label = self.model.labels[index]
        confidence = float(probabilities[index].item())
        steering = {"LEFT": 1.0, "STRAIGHT": 0.0, "RIGHT": -1.0, "STOP": 0.0}[label]
        speed = 0.0 if label == "STOP" else 1.0
        return TwistCommand(speed, steering * self.steering_rate, label, confidence)


if rclpy is not None:
    class VisualSteeringNode(Node):
        """Stop on invalid images, inference errors or stale input / 输入异常或超时即停车。"""

        def __init__(self) -> None:
            super().__init__("visual_steering_node")
            image_topic = str(self.declare_parameter("image_topic", "/front_rgb/image_raw").value)
            command_topic = str(self.declare_parameter("command_topic", "/cmd_vel").value)
            self.image_timeout = float(self.declare_parameter("image_timeout_s", 0.5).value)
            if not math.isfinite(self.image_timeout) or self.image_timeout <= 0.0:
                raise ValueError("image_timeout_s must be positive / 图像超时必须为正数")
            self.publisher = self.create_publisher(Twist, command_topic, 10)
            self._last_image_time: float | None = None
            self.core: VisualSteeringCore | None = None
            self.bridge = CvBridge()
            checkpoint_value = str(self.declare_parameter("checkpoint", "").value).strip()
            try:
                checkpoint = Path(checkpoint_value)
                if not checkpoint_value or not checkpoint.is_file():
                    raise FileNotFoundError(f"checkpoint not found / 找不到模型权重: {checkpoint}")
                self.core = VisualSteeringCore(
                    checkpoint, float(self.declare_parameter("steering_rate", 0.65).value)
                )
            except Exception as error:
                self.get_logger().error(f"Inference disabled; zero velocity / 推理停用并保持零速: {error}")
            self.subscription = self.create_subscription(RosImage, image_topic, self.on_image, 10)
            self.watchdog = self.create_timer(min(0.1, self.image_timeout / 2.0), self.check_timeout)
            self.publish_zero()

        def publish_zero(self) -> None:
            self.publisher.publish(Twist())

        def check_timeout(self) -> None:
            if (self._last_image_time is None
                    or time.monotonic() - self._last_image_time >= self.image_timeout):
                self.publish_zero()

        def on_image(self, message: RosImage) -> None:
            received_at = time.monotonic()
            try:
                if self.core is None:
                    self.publish_zero()
                    return
                if message.width <= 0 or message.height <= 0 or not message.data:
                    raise ValueError("empty image / 空图像")
                array = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
                if array.shape != (message.height, message.width, 3) or array.dtype != np.uint8:
                    raise ValueError("invalid RGB image shape or type / RGB 图像尺寸或类型错误")
                command = self.core.predict(Image.fromarray(array))
                if time.monotonic() - received_at >= self.image_timeout:
                    raise TimeoutError("inference exceeded image timeout / 推理超过图像超时时间")
                if not all(math.isfinite(value) for value in (command.linear_x, command.angular_z)):
                    raise ValueError("non-finite command / 控制指令包含非有限值")
                twist = Twist()
                twist.linear.x = command.linear_x
                twist.angular.z = command.angular_z
                self.publisher.publish(twist)
                self._last_image_time = received_at
            except Exception as error:
                self._last_image_time = None
                self.publish_zero()
                self.get_logger().warning(f"Image rejected; zero velocity / 图像处理失败并停车: {error}")


def ros_main(args: list[str] | None = None) -> int:
    if rclpy is None:
        raise RuntimeError("rclpy and cv_bridge are required / 运行 ROS 2 需要 rclpy 与 cv_bridge")
    node = None
    rclpy.init(args=args)
    try:
        node = VisualSteeringNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.publish_zero()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def replay_directory(checkpoint: Path, image_dir: Path) -> dict[str, object]:
    core = VisualSteeringCore(checkpoint)
    frames = sorted(image_dir.glob("*.png"))
    if not frames:
        raise ValueError("no PNG frames found / 未找到 PNG 图像")
    commands = []
    for path in frames:
        with Image.open(path) as image:
            commands.append(core.predict(image))
    return {"frames": len(commands),
            "labels": {label: sum(c.label == label for c in commands) for label in core.model.labels},
            "commands": [command.__dict__ for command in commands]}


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay RGB images / 回放 RGB 图像")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(replay_directory(args.checkpoint.resolve(), args.images.resolve()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

