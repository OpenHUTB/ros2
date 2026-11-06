"""
classify_node.py — 订阅声呐图像，CNN 推理，发布分类结果。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import torch

from . import common
from .cnn_model import SonarCNN


CLASS_NAMES = ["礁石(rock)", "管道(pipe)", "残骸(wreck)"]


class ClassifyNode(Node):
    def __init__(self):
        super().__init__("rov_sonar_classify")
        wpath = common.find_weights_path(self.declare_parameter("weights_path", "").value)
        if not wpath.exists():
            raise FileNotFoundError(
                f"找不到 CNN 权重 {wpath}。请先运行: python3 main.py train"
            )
        self.net = SonarCNN()
        self.net.load_state_dict(torch.load(wpath, map_location="cpu"))
        self.net.eval()
        self.get_logger().info(f"SonarCNN 已加载: {wpath}")

        self.create_subscription(Image, "/rov_sonar/image_raw", self._on_img, 10)
        self.pub = self.create_publisher(String, "/rov_sonar/classification", 10)

    def _on_img(self, msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        x = torch.tensor(arr, dtype=torch.float32).unsqueeze(0).unsqueeze(0) / 255.0
        with torch.no_grad():
            out = self.net(x)
            probs = torch.softmax(out, dim=1).squeeze()
            pred = out.argmax(dim=1).item()
        result = String()
        result.data = f"class={CLASS_NAMES[pred]}  probs=[{','.join(f'{p:.2f}' for p in probs.tolist())}]"
        self.pub.publish(result)


def main():
    rclpy.init()
    node = ClassifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
