#!/usr/bin/env python3
"""HSV 颜色阈值目标检测：把红色目标相对画面中心的偏移与像素面积发布到 /vision/target。

PointStamped 没有顶层 x/y/z，坐标与面积都在 .point 子消息里：
- point.x = 质心相对画面中心的水平偏移（像素，右正）
- point.y = 质心相对画面中心的垂直偏移（像素，下正）
- point.z = 目标轮廓像素面积（未检出时为 0）

同时把带检测框/质心/数值的调试图发布到 /vision/debug_image（bgr8），
可用 rqt_image_view 或 ros2 run image_view 等工具直接查看，便于验收与录屏。

参数：
- min_area：有效目标的最小轮廓面积，默认 300 px^2
- hsv_*：红色双区间范围（如现场光照不同可调整）
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class ColorTracker(Node):
    """订阅 /camera/image_raw，发布 /vision/target 与 /vision/debug_image。"""

    def __init__(self):
        super().__init__('color_tracker')
        self.declare_parameter('min_area', 300.0)
        self.declare_parameter('hsv_lo1', [0, 100, 100])
        self.declare_parameter('hsv_hi1', [10, 255, 255])
        self.declare_parameter('hsv_lo2', [160, 100, 100])
        self.declare_parameter('hsv_hi2', [180, 255, 255])
        self.declare_parameter('publish_debug_image', True)

        self.min_area = float(self.get_parameter('min_area').value)
        self.lo1 = tuple(int(v) for v in self.get_parameter('hsv_lo1').value)
        self.hi1 = tuple(int(v) for v in self.get_parameter('hsv_hi1').value)
        self.lo2 = tuple(int(v) for v in self.get_parameter('hsv_lo2').value)
        self.hi2 = tuple(int(v) for v in self.get_parameter('hsv_hi2').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(PointStamped, '/vision/target', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 10)
        self.bridge = CvBridge()
        self.get_logger().info(
            'color_tracker: HSV 红色双区间检测，min_area=%.0f px^2' % self.min_area)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # 单帧解码失败不应打断节点
            self.get_logger().warn('图像转换失败，跳过该帧: %s' % exc)
            return

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.lo1, self.hi1),
            cv2.inRange(hsv, self.lo2, self.hi2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'camera'
        p.point.z = 0.0

        debug = frame.copy()
        cv2.drawMarker(debug, (w // 2, h // 2), (255, 200, 0),
                       cv2.MARKER_CROSS, 24, 1)

        cnts, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_area:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']
                    p.point.x = float(cx - w / 2.0)
                    p.point.y = float(cy - h / 2.0)
                    p.point.z = float(area)
                    cv2.drawContours(debug, [c], -1, (0, 255, 0), 2)
                    cv2.circle(debug, (int(cx), int(cy)), 4, (0, 255, 255), -1)
                    cv2.line(debug, (w // 2, h // 2), (int(cx), int(cy)),
                             (0, 200, 255), 2)
                    cv2.putText(debug,
                                'dx=%+.0f dy=%+.0f area=%.0f' % (p.point.x, p.point.y, area),
                                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 255, 0), 2)
        self.pub.publish(p)

        if self.publish_debug:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
