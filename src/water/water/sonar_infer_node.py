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
            torch.nn.Linear(64 * 60 * 80, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, num_classes)
        )
    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

class SonarInferNode(Node):
    def __init__(self):
        super().__init__('sonar_infer_node')
        self.bridge = CvBridge()
        self.model = SonarCNN(num_classes=4)
        self.model.eval()
        self.class_names = ["rock", "wreck", "pipe", "sand"]
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((480,640)),
            transforms.ToTensor(),
        ])
        # 订阅正确话题
        self.sub = self.create_subscription(
            Image,
            "/sonar/sonar_camera/image_raw",
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(String, "/sonar/result", 10)
        self.get_logger().info("声呐图像识别节点已启动，等待图像...")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            tensor_img = self.transform(img_rgb).unsqueeze(0)
            with torch.no_grad():
                out = self.model(tensor_img)
                pred_idx = torch.argmax(out, dim=1).item()
            res_name = self.class_names[pred_idx]
            msg_out = String()
            msg_out.data = f"识别结果：{res_name}"
            self.pub.publish(msg_out)
            self.get_logger().info(f"Sonar detect: {res_name}")
        except Exception as e:
            self.get_logger().error(f"处理图像出错：{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SonarInferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
