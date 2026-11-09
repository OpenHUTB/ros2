import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.transforms as transforms

class SonarCNN(torch.nn.Module):
    def __init__(self, num_classes=4):
        super().__init__()
        self.features = torch.nn.Sequential(
            torch.nn.Conv2d(3, 16, 3, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(2),
            torch.nn.Conv2d(16, 32, 3, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(2),
            torch.nn.Conv2d(32, 64, 3, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(2),
        )
        self.classifier = torch.nn.Sequential(
            torch.nn.Flatten(),
            torch.nn.Linear(64 * 28 * 28, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, num_classes)
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x


class SonarInferNode(Node):
    def __init__(self):
        super().__init__("sonar_infer_node")
        self.bridge = CvBridge()
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor()
        ])
        # 模型初始化 + 加载权重（重点新增）
        self.model = SonarCNN(num_classes=4)
        self.model.load_state_dict(torch.load("sonar_cnn_weights.pth", map_location=torch.device("cpu")))
        self.model.eval()

        self.sub_img = self.create_subscription(
            Image,
            "/sonar_image",
            self.image_callback,
            10
        )
        self.pub_result = self.create_publisher(String, "/sonar_rec_result", 10)
        self.class_names = ["class0", "class1", "class2", "class3"]
        self.get_logger().info("声呐CNN推理节点启动，权重加载成功")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            tensor_img = self.transform(img_rgb).unsqueeze(0)
            with torch.no_grad():
                out = self.model(tensor_img)
                pred_idx = torch.argmax(out, dim=1).item()
            res_msg = String()
            res_msg.data = self.class_names[pred_idx]
            self.pub_result.publish(res_msg)
            self.get_logger().info(f"识别结果：{self.class_names[pred_idx]}")
        except Exception as e:
            self.get_logger().error(f"推理出错：{str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SonarInferNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
