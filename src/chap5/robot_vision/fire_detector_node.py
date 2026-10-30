import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FireDetectorNode(Node):
    def __init__(self):
        super().__init__('fire_detector_node')
        self.bridge = CvBridge()
        # 订阅相机图像话题
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("火焰识别告警节点已启动，等待图像输入")

    def image_callback(self, msg):
        # ROS图像转为OpenCV图像
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # 火焰HSV阈值范围
        lower_red = np.array([0, 120, 120])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            self.get_logger().info("【告警】检测到火焰目标！")
        else:
            self.get_logger().info("未检测到火焰")

def main():
    rclpy.init()
    node = FireDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
