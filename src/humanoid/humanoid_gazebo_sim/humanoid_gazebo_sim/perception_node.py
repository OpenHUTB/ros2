#!/usr/bin/env python3
"""人形机器人感知节点：HSV颜色分割识别红色目标"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2, numpy as np
from cv_bridge import CvBridge

class HumanoidPerception(Node):
    def __init__(self):
        super().__init__('humanoid_perception')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/humanoid/camera/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Point, '/humanoid/target_pos', 10)
        self.lower1 = np.array([0, 80, 50])
        self.upper1 = np.array([10, 255, 255])
        self.lower2 = np.array([170, 80, 50])
        self.upper2 = np.array([180, 255, 255])

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, self.lower1, self.upper1),
                              cv2.inRange(hsv, self.lower2, self.upper2))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx, cy = M['m10']/M['m00'], M['m01']/M['m00']
                h, w = frame.shape[:2]
                p = Point()
                p.x = (cx - w/2) / w
                p.y = (cy - h/2) / h
                p.z = 1.0
                self.pub.publish(p)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
