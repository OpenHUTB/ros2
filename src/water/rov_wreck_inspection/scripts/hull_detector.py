#!/usr/bin/env python3
"""沉船船体结构特征视觉检测节点。

订阅 ROV 前视相机图像，用 HSV 颜色分割提取两类目标：

1. 船体锈蚀区域（红棕色）——判断船体是否在视野内，并给出船体像素质心；
2. 结构特征标记板（高亮黄色板）——给出归一化像平面坐标与单目测距结果。

测距使用针孔模型 ``d = f * W / w``：``W`` 为标记板真实边长（0.55 m），
``w = max(bw, bh)`` 为像平面上检测框的长边像素数，``f = (width/2) / tan(hfov/2)``
由相机内参推出。斜视时该近似存在系统性误差，结果仅作为单目估计距离。
"""
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# 黄色结构标记板（自发光，HSV 区间可以取得很窄）
MARKER_LO = np.array([18, 90, 110])
MARKER_HI = np.array([40, 255, 255])
# 锈蚀船体（红棕色，饱和度较高、亮度中等）
HULL_LO = np.array([0, 80, 30])
HULL_HI = np.array([16, 255, 200])


class HullDetector(Node):
    def __init__(self):
        super().__init__("hull_detector")
        # marker_size / camera_hfov 只在 config/inspection.yaml 维护，避免两处不一致
        default_cfg = os.path.join(
            get_package_share_directory("rov_wreck_inspection"),
            "config", "inspection.yaml")
        cfg_path = self.declare_parameter("config_file", default_cfg).value
        self.marker_size = 0.55
        self.hfov = 1.22
        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            self.marker_size = float(cfg.get("marker_size", self.marker_size))
            self.hfov = float(cfg.get("camera_hfov", self.hfov))
        except OSError:
            self.get_logger().warn(
                f"无法读取配置文件 {cfg_path}，使用内置测距参数")
        self.min_area = int(self.declare_parameter("min_marker_area", 60).value)
        self.encoding_warned = False

        self.image_sub = self.create_subscription(
            Image, "/rov/camera/image_raw", self.on_image, 10)
        self.detect_pub = self.create_publisher(
            Float32MultiArray, "/rov/hull_detection", 10)
        self.image_pub = self.create_publisher(
            Image, "/rov/camera/image_detected", 10)

        self.dist_filt = None
        self.frames = 0

        self.get_logger().info(
            f"船体视觉检测节点启动：标记板 {self.marker_size} m，"
            f"水平视场 {math.degrees(self.hfov):.1f}°")

    # ------------------------------------------------------------------
    def on_image(self, msg):
        img = self.to_bgr(msg)
        if img is None:
            if not self.encoding_warned:
                self.get_logger().warn(
                    f"不支持的图像编码 {msg.encoding}，仅支持 rgb8/bgr8")
                self.encoding_warned = True
            return
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 1) 结构标记板：取最靠近画面中心的黄色连通域
        mask = cv2.inRange(hsv, MARKER_LO, MARKER_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        marker_count = len(contours)

        marker_valid = 0.0
        marker_u = marker_v = marker_dist = marker_w = 0.0
        if contours:
            def center_dist(c):
                (cx_, cy_), _, _ = cv2.minAreaRect(c)
                return (cx_ - w / 2.0) ** 2 + (cy_ - h / 2.0) ** 2

            best = min(contours, key=center_dist)
            (cx, cy), (bw, bh), _ = cv2.minAreaRect(best)
            marker_valid = 1.0
            marker_u = (cx / w) - 0.5           # 归一化像平面坐标，右为正
            marker_v = (cy / h) - 0.5           # 下为正
            marker_w = float(max(bw, bh))
            focal = (w / 2.0) / math.tan(self.hfov / 2.0)
            dist = focal * self.marker_size / max(marker_w, 1e-3)
            # 一阶低通，抑制单帧抖动
            self.dist_filt = dist if self.dist_filt is None else (
                0.7 * self.dist_filt + 0.3 * dist)
            marker_dist = self.dist_filt
            cv2.drawContours(img, [best], -1, (0, 255, 255), 2)
            cv2.circle(img, (int(cx), int(cy)), 4, (0, 255, 255), -1)

        # 2) 船体锈蚀区域
        hull_mask = cv2.inRange(hsv, HULL_LO, HULL_HI)
        hull_mask = cv2.morphologyEx(
            hull_mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
        hc, _ = cv2.findContours(
            hull_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hull_valid = 0.0
        hull_u = hull_v = hull_ratio = 0.0
        if hc:
            biggest = max(hc, key=cv2.contourArea)
            area = cv2.contourArea(biggest)
            hull_ratio = area / float(w * h)
            if hull_ratio > 0.04:
                hull_valid = 1.0
                x, y, bw2, bh2 = cv2.boundingRect(biggest)
                hull_u = ((x + bw2 / 2.0) / w) - 0.5
                hull_v = ((y + bh2 / 2.0) / h) - 0.5
                cv2.rectangle(img, (x, y), (x + bw2, y + bh2), (0, 140, 255), 1)

        # 叠加准星与文字，便于 RViz 中直接观察
        cv2.line(img, (w // 2 - 18, h // 2), (w // 2 + 18, h // 2), (200, 200, 200), 1)
        cv2.line(img, (w // 2, h // 2 - 18), (w // 2, h // 2 + 18), (200, 200, 200), 1)
        if marker_valid:
            text = f"STRUCT MARKER  dist={marker_dist:.2f}m  u={marker_u:+.2f}"
            color = (0, 255, 255)
        elif hull_valid:
            text = "HULL DETECTED - no structural marker"
            color = (0, 140, 255)
        else:
            text = "SEARCHING HULL ..."
            color = (180, 180, 180)
        cv2.putText(img, text, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)
        cv2.putText(img, f"hull={hull_valid:.0f} markers={marker_count}",
                    (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        arr = Float32MultiArray()
        arr.data = [hull_valid, hull_u, hull_v, hull_ratio,
                    marker_valid, marker_u, marker_v, marker_dist,
                    marker_w, float(marker_count)]
        self.detect_pub.publish(arr)
        self.image_pub.publish(self.to_msg(img, msg.header))

        self.frames += 1
        if self.frames % 300 == 0:
            self.get_logger().info(
                f"检测统计：船体={int(hull_valid)}，标记板={marker_count}，"
                f"距离={marker_dist:.2f} m")

    # ------------------------------------------------------------------
    @staticmethod
    def to_bgr(msg):
        """把 Gazebo 相机图像转成 OpenCV BGR 数组，避免依赖 cv_bridge。"""
        if msg.encoding not in ("rgb8", "bgr8"):
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = buf.reshape((msg.height, msg.step // 3, 3))[:, :msg.width, :]
        except ValueError:
            return None
        img = np.ascontiguousarray(img)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    @staticmethod
    def to_msg(img, header):
        msg = Image()
        msg.header = header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg


def main():
    rclpy.init()
    node = HullDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
