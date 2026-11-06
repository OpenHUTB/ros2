#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionDetectNode(Node):
    def __init__(self):
        super().__init__("vision_detect_node")
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.target_pub = self.create_publisher(PointStamped, "/leader_target", 10)
        self.img_w = 640
        self.img_h = 480

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        # 识别领航机器人红色标记
        lower = np.array([0, 120, 120])
        upper = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        target_msg = PointStamped()
        target_msg.header = msg.header
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 1:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                target_msg.point.x = float(cx)
                target_msg.point.y = float(cy)
                target_msg.point.z = float(cv2.contourArea(c))
        self.target_pub.publish(target_msg)

def main():
    rclpy.init()
    node = VisionDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
