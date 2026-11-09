#!/usr/bin/env python3
"""水下ROV感知节点：HSV颜色分割识别水下红色目标"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge


class RovPerception(Node):
    def __init__(self):
        super().__init__('rov_perception')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/rov/camera/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Point, '/rov/target_pos', 10)
        # HSV红色范围（水下红色偏暗）
        self.lower1 = np.array([0, 80, 30])
        self.upper1 = np.array([10, 255, 255])
        self.lower2 = np.array([170, 80, 30])
        self.upper2 = np.array([180, 255, 255])
        self.get_logger().info('ROV感知节点已启动，等待摄像头图像...')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                h, w = frame.shape[:2]
                point = Point()
                point.x = (cx - w / 2) / w  # 归一化x偏移
                point.y = (cy - h / 2) / h  # 归一化y偏移
                point.z = 1.0  # 检测到目标
                self.pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = RovPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
