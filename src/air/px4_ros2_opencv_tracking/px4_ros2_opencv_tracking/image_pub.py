#!/usr/bin/env python3
"""合成相机图像源：发布 640x480、黑底红色移动圆到 /camera/image_raw。

gz_x500 默认模型没有相机，因此本节点用于给视觉追踪提供可复现的图像输入。
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePub(Node):
    """以 10 Hz 发布一张合成图像，红色目标在画面内做李萨如运动。"""

    def __init__(self):
        super().__init__('image_pub')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.t = 0.0
        self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cx = int(320 + 200 * np.sin(self.t * 0.8))
        cy = int(240 + 150 * np.sin(self.t * 0.5 + 1.0))
        cv2.circle(img, (cx, cy), 35, (0, 0, 255), -1)
        self.t += 0.1
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
