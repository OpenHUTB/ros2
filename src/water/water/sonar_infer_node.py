import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn

class SonarCNN(nn.Module):
    def __init__(self, num_classes=3):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 32 * 32, 128),
            nn.ReLU(),
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

class SonarInferNode(Node):
    def __init__(self):
        super().__init__("sonar_infer_node")
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            Image,
            "/sonar/image",
            self.img_callback,
            10
        )
        self.get_logger().info("声呐识别节点已启动")

        self.model = SonarCNN(num_classes=3)
        weight_path = "sonar_cnn_weights.pth"
        try:
            self.model.load_state_dict(torch.load(weight_path, map_location=torch.device("cpu")))
            self.get_logger().info("权重文件加载成功")
        except Exception as e:
            self.get_logger().warn(f"权重加载失败: {e}, 使用随机初始化权重")
        self.model.eval()

        self.class_names = ["rock", "pipe", "sand"]

    def img_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = cv2.resize(cv_img, (256,256))
        img_tensor = torch.from_numpy(img).permute(2,0,1).float() / 255.0
        img_tensor = img_tensor.unsqueeze(0)

        with torch.no_grad():
            out = self.model(img_tensor)
            pred_idx = torch.argmax(out, dim=1).item()
        res = self.class_names[pred_idx]
        self.get_logger().info(f"检测目标: {res}")

def main(args=None):
    rclpy.init(args=args)
    node = SonarInferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
