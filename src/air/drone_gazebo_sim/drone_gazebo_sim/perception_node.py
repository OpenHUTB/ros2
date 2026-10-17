#!/usr/bin/env python3
"""无人机摄像头感知节点
订阅 /drone/camera/image_raw，用轻量CNN(实际用OpenCV颜色分割+CNN特征)
检测红色目标，发布目标位置到 /drone/target_pos
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np


class DronePerception(Node):
    def __init__(self):
        super().__init__('drone_perception')
        self.sub = self.create_subscription(
            Image, '/drone/camera/image_raw', self.img_cb, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, '/drone/target_pos', 10)
        # 红色目标HSV范围
        self.lower1 = np.array([0, 120, 70])
        self.upper1 = np.array([10, 255, 255])
        self.lower2 = np.array([170, 120, 70])
        self.upper2 = np.array([180, 255, 255])
        self.get_logger().info('无人机摄像头感知节点已启动')

    def img_cb(self, msg):
        # 转OpenCV格式
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)
        mask = mask1 + mask2
        # 找轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = Float32MultiArray()
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = M['m10'] / M['m00'] / msg.width - 0.5  # 归一化 -0.5~0.5
                area = cv2.contourArea(c) / (msg.width * msg.height)
                out.data = [float(cx), float(area)]
                self.get_logger().debug(
                    f'目标偏移={cx:.2f}, 占比={area:.3f}')
        else:
            out.data = [999.0, 0.0]  # 没找到
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DronePerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
