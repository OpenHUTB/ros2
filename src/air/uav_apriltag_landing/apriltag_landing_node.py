#!/usr/bin/env python3
"""
AprilTag 视觉精准降落节点.

流程:
  SEARCH  : 在预设高度悬停, 等待下视相机看到 AprilTag
  ALIGN   : 根据标签像素偏差, 把无人机平移到标签正上方
  DESCEND : 对齐误差小于阈值后缓慢降高, 持续精对准
  LANDED  : 高度接近地面时触发, 完成降落

话题约定 (与 AirSim/仿真桥接模块对接):
  订阅: /drone/down_camera/image_raw   sensor_msgs/Image
  订阅: /drone/height                  std_msgs/Float64 (相对地面高度, m)
  发布: /drone/setpoint_position        geometry_msgs/PointStamped
  发布: /drone/landing_state           std_msgs/String
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String

try:
    from dt_apriltags import Detector
    HAS_DT_APRILTAGS = True
except ImportError:
    HAS_DT_APRILTAGS = False


ST_SEARCH = "SEARCH"
ST_ALIGN = "ALIGN"
ST_DESCEND = "DESCEND"
ST_LANDED = "LANDED"


class AprilTagLandingNode(Node):
    def __init__(self):
        super().__init__("apriltag_landing")

        self.declare_parameter("hover_height", 5.0)
        self.declare_parameter("align_pixel_tol", 15.0)
        self.declare_parameter("descend_step", 0.5)
        self.declare_parameter("landed_height", 0.1)
        self.declare_parameter("cam_fx", 500.0)
        self.declare_parameter("cam_fy", 500.0)
        self.declare_parameter("cam_cx", 320.0)
        self.declare_parameter("cam_cy", 240.0)
        self.declare_parameter("tag_family", "tag36h11")

        self.hover_height = self.get_parameter("hover_height").value
        self.align_tol = float(self.get_parameter("align_pixel_tol").value)
        self.descend_step = float(self.get_parameter("descend_step").value)
        self.landed_h = float(self.get_parameter("landed_height").value)
        self.fx = float(self.get_parameter("cam_fx").value)
        self.fy = float(self.get_parameter("cam_fy").value)
        self.cx = float(self.get_parameter("cam_cx").value)
        self.cy = float(self.get_parameter("cam_cy").value)

        self.state = ST_SEARCH
        self.height = 0.0
        self.tag_detected = False
        self.tag_pixel = (0.0, 0.0)
        self.current_target = (0.0, 0.0, self.hover_height)

        self.bridge = CvBridge()

        if HAS_DT_APRILTAGS:
            self.detector = Detector(
                families=self.get_parameter("tag_family").value,
                nthreads=1,
                quad_decimate=1.0,
                search_value_hamming=1,
            )
            self.get_logger().info("dt_apriltags 已加载")
        else:
            self.detector = None
            self.get_logger().warn(
                "未安装 dt_apriltags, 回退到亮斑演示模式 "
                "(pip install dt-apriltags 启用完整检测)"
            )

        self.img_sub = self.create_subscription(
            Image, "/drone/down_camera/image_raw", self._img_cb, 10)
        self.hgt_sub = self.create_subscription(
            Float64, "/drone/height", self._hgt_cb, 10)
        self.target_pub = self.create_publisher(
            PointStamped, "/drone/setpoint_position", 10)
        self.state_pub = self.create_publisher(
            String, "/drone/landing_state", 10)

        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f"AprilTag 降落节点已启动, 悬停高度={self.hover_height}m")

    def _hgt_cb(self, msg: Float64):
        self.height = float(msg.data)

    def _img_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        self.tag_detected = False
        if self.detector is not None:
            results = self.detector.detect(
                frame, estimate_tag_pose=False,
                camera_params=(self.fx, self.fy, self.cx, self.cy),
                tag_size=0.16)
            if results:
                r = results[0]
                self.tag_detected = True
                self.tag_pixel = (float(r.center[0]), float(r.center[1]))
        else:
            _, th = cv2.threshold(frame, 200, 255, cv2.THRESH_BINARY)
            cnts, _ = cv2.findContours(
                th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    self.tag_detected = True
                    self.tag_pixel = (M["m10"] / M["m00"],
                                      M["m01"] / M["m00"])

    def _control_loop(self):
        dx_px = self.tag_pixel[0] - self.cx
        dy_px = self.tag_pixel[1] - self.cy
        h = max(self.height, 0.3)

        if self.state == ST_SEARCH:
            if self.tag_detected:
                self.state = ST_ALIGN
            else:
                self._publish(*self.current_target)

        elif self.state == ST_ALIGN:
            if not self.tag_detected:
                self.state = ST_SEARCH
            else:
                tx = self.current_target[0] + dx_px * h / self.fx
                ty = self.current_target[1] + dy_px * h / self.fy
                self.current_target = (tx, ty, self.hover_height)
                self._publish(tx, ty, self.hover_height)
                if abs(dx_px) < self.align_tol and abs(dy_px) < self.align_tol:
                    self.state = ST_DESCEND
                    self.get_logger().info("对齐完成, 开始降高")

        elif self.state == ST_DESCEND:
            if not self.tag_detected:
                self.state = ST_ALIGN
            else:
                tx = self.current_target[0] + dx_px * h / self.fx * 0.5
                ty = self.current_target[1] + dy_px * h / self.fy * 0.5
                new_h = max(self.height - self.descend_step, self.landed_h)
                self.current_target = (tx, ty, new_h)
                self._publish(tx, ty, new_h)
                if self.height <= self.landed_h + 0.05:
                    self.state = ST_LANDED
                    self.get_logger().info("已接地, 降落完成")

        elif self.state == ST_LANDED:
            self._publish(self.current_target[0], self.current_target[1], 0.0)

        self.state_pub.publish(String(data=self.state))

    def _publish(self, x, y, z):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
