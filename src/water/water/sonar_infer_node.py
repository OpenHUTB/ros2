import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F

# ---------------------- CNN模型定义 ----------------------
class SonarCNN(nn.Module):
    def __init__(self, num_classes=3):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 16, 3, padding=1)
        self.conv2 = nn.Conv2d(16, 32, 3, padding=1)
        self.pool = nn.MaxPool2d(2,2)
        self.fc1 = nn.Linear(32 * 32 * 32, 128)
        self.fc2 = nn.Linear(128, num_classes)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = torch.flatten(x, 1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

# 类别名称，可根据你的数据集修改
CLASS_NAMES = ["rock", "fish", "wreck"]

# ---------------------- ROS2节点 ----------------------
class SonarInferNode(Node):
    def __init__(self):
        super().__init__('sonar_infer_node')
        self.bridge = CvBridge()

        # 加载CNN模型
        self.model = SonarCNN(num_classes=3)
        self.model.eval()
        self.get_logger().info("Sonar CNN model loaded")

        # 订阅Gazebo相机图像话题
        self.sub_img = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Sonar inference node started, waiting for image...")

    def image_callback(self, msg):
        try:
            # ROS图像转OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 转为灰度图（声呐图像是单通道灰度）
            gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            # 缩放到模型输入尺寸
            resized_img = cv2.resize(gray_img, (128,128))

            # 图像转tensor，适配网络输入 [batch, channel, H, W]
            tensor_img = torch.from_numpy(resized_img).float().unsqueeze(0).unsqueeze(0) / 255.0

            # CNN推理
            with torch.no_grad():
                out = self.model(tensor_img)
                pred_idx = torch.argmax(out, dim=1).item()
                pred_class = CLASS_NAMES[pred_idx]

            # 在ROS日志打印识别结果
            self.get_logger().info(f"【识别结果】目标类别: {pred_class}")

            # 可视化窗口（可选）
            cv2.putText(cv_img, pred_class, (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
            cv2.imshow("Sonar Image", cv_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image process error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = SonarInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
