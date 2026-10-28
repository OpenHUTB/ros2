#!/usr/bin/env python3
"""HSV 颜色阈值目标检测：把红色目标相对画面中心的偏移与像素面积发布到 /vision/target。

PointStamped 没有顶层 x/y/z，坐标与面积都在 .point 子消息里：
point.x = 质心相对画面中心的水平偏移（像素，右正）
point.y = 质心相对画面中心的垂直偏移（像素，下正）
point.z = 目标轮廓像素面积（未检出时为 0）
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class ColorTracker(Node):
    """订阅 /camera/image_raw，发布 /vision/target。"""

    def __init__(self):
        super().__init__('color_tracker')
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(PointStamped, '/vision/target', 10)
        self.bridge = CvBridge()
        self.h, self.w = 480, 640
        self.min_area = 300.0

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # 单帧解码失败不应打断节点
            self.get_logger().warn(f'图像转换失败，跳过滤帧: {exc}')
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'camera'
        p.point.z = 0.0
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_area:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    p.point.x = float(M['m10'] / M['m00'] - self.w / 2.0)
                    p.point.y = float(M['m01'] / M['m00'] - self.h / 2.0)
                    p.point.z = float(area)
        self.pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
