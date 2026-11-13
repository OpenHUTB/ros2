#!/usr/bin/env python3
"""合成相机图像源（**非真实相机**，用于可复现的视觉算法演示）。

本节点用 OpenCV 画一个黑底红色圆，并按李萨如曲线移动，发布到
/camera/image_raw（640x480, bgr8, 默认 10 Hz），用于演示
`color_tracker` 的 HSV 颜色检测与 `offboard_control` 的 Offboard 速度控制闭环。

注意：PX4 的 gz_x500 机型默认没有相机，因此该图像与无人机在 Gazebo 中的
运动**没有关系**，本模块是「开环视觉输入」演示；如需真实视觉伺服，请改用
gz_x500_mono_cam 等带相机的机型并通过 ros_gz_bridge 桥出相机话题。

参数（可在 launch 中覆盖）：
- image_width / image_height：图像尺寸，默认 640x480
- radius_px：红圆半径，默认 35 px（contourArea 实测约 3750 px^2）
- rate_hz：发布频率，默认 10 Hz
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePub(Node):
    """以固定频率发布一张合成图像，红色目标在画面内做李萨如运动。"""

    def __init__(self):
        super().__init__('image_pub')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('radius_px', 35)
        self.declare_parameter('rate_hz', 10.0)

        self.w = int(self.get_parameter('image_width').value)
        self.h = int(self.get_parameter('image_height').value)
        self.radius = int(self.get_parameter('radius_px').value)
        rate = float(self.get_parameter('rate_hz').value)

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / max(rate, 1.0), self.publish_image)
        self.get_logger().info(
            'image_pub: 合成图像 %dx%d, r=%d px, %.1f Hz（非真实相机）'
            % (self.w, self.h, self.radius, rate))

    def publish_image(self):
        # 用定时器时钟计算相位，避免旧实现里 self.t += 0.1 的累计漂移
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        cx = int(self.w / 2 + 0.31 * self.w * np.sin(t * 0.8))
        cy = int(self.h / 2 + 0.31 * self.h * np.sin(t * 0.5 + 1.0))
        cv2.circle(img, (cx, cy), self.radius, (0, 0, 255), -1)
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
