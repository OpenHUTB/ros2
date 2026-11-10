#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class VisionDetectNode(Node):
    def __init__(self):
        super().__init__("vision_detect_node")
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.target_pub = self.create_publisher(PointStamped, "/leader_target", 10)

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        # 红色在 HSV 中横跨 0 度，需要 [0,10] 与 [170,180] 两段取并集
        lower1 = np.array([0, 120, 120])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 120, 120])
        upper2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
