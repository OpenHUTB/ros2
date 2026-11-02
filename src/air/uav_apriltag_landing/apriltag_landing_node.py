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
  订阅: /drone/odom                    geometry_msgs/PointStamped (当前位置, 可选)
  发布: /drone/setpoint_position        geometry_msgs/PointStamped
  发布: /drone/landing_state           std_msgs/String

相机假设: 下视相机光轴垂直向下, 图像 x 轴与机体 x 轴同向,
  图像 y 轴与机体 y 轴同向 (机身前端朝图像上方)。
  若安装方向不同, 需在 launch 中调整 flip_x / flip_y 参数。
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

from landing_math import pixel_to_ground, ST_SEARCH, ST_ALIGN, ST_DESCEND, ST_LANDED


class AprilTagLandingNode(Node):
    def __init__(self):
        super().__init__("apriltag_landing")

        # ---- 可配置参数 ----
        self.declare_parameter("hover_height", 5.0)
        self.declare_parameter("align_pixel_tol", 15.0)
        self.declare_parameter("descend_step", 0.3)
        self.declare_parameter("descend_speed", 0.5)
        self.declare_parameter("landed_height", 0.1)
        self.declare_parameter("cam_fx", 500.0)
        self.declare_parameter("cam_fy", 500.0)
        self.declare_parameter("cam_cx", 320.0)
        self.declare_parameter("cam_cy", 240.0)
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("tag_size", 0.16)
        self.declare_parameter("target_tag_id", -1)
        self.declare_parameter("flip_x", False)
        self.declare_parameter("flip_y", False)
        self.declare_parameter("img_timeout", 1.0)
        self.declare_parameter("height_timeout", 2.0)

        self.hover_height = float(self.get_parameter("hover_height").value)
        self.align_tol = float(self.get_parameter("align_pixel_tol").value)
        self.descend_step = float(self.get_parameter("descend_step").value)
        self.descend_speed = float(self.get_parameter("descend_speed").value)
        self.landed_h = float(self.get_parameter("landed_height").value)
        self.fx = float(self.get_parameter("cam_fx").value)
        self.fy = float(self.get_parameter("cam_fy").value)
        self.cx = float(self.get_parameter("cam_cx").value)
        self.cy = float(self.get_parameter("cam_cy").value)
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.target_tag_id = int(self.get_parameter("target_tag_id").value)
        self.flip_x = bool(self.get_parameter("flip_x").value)
        self.flip_y = bool(self.get_parameter("flip_y").value)
        self.img_timeout = float(self.get_parameter("img_timeout").value)
        self.height_timeout = float(self.get_parameter("height_timeout").value)

        # ---- 状态 ----
        self.state = ST_SEARCH
        self.height = 0.0
        self.height_init = False
        self.tag_detected = False
        self.tag_pixel = (0.0, 0.0)
        self.current_target = (0.0, 0.0, self.hover_height)
        self.pose_received = False
        self.last_img_time = self.get_clock().now()
        self.last_height_time = self.get_clock().now()

        self.bridge = CvBridge()

        if HAS_DT_APRILTAGS:
            self.detector = Detector(
                families=self.get_parameter("tag_family").value,
                nthreads=1,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
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
        self.pose_sub = self.create_subscription(
            PointStamped, "/drone/odom", self._pose_cb, 10)
        self.target_pub = self.create_publisher(
            PointStamped, "/drone/setpoint_position", 10)
        self.state_pub = self.create_publisher(
            String, "/drone/landing_state", 10)

        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f"AprilTag 降落节点已启动, 悬停高度={self.hover_height}m")

    def _hgt_cb(self, msg: Float64):
        self.height = float(msg.data)
        self.height_init = True
        self.last_height_time = self.get_clock().now()

    def _pose_cb(self, msg: PointStamped):
        if not self.pose_received:
            self.current_target = (msg.point.x, msg.point.y, self.hover_height)
            self.pose_received = True
            self.get_logger().info(
                f"已收到当前位姿, 初始目标=({msg.point.x:.2f},{msg.point.y:.2f})")

    def _img_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        self.tag_detected = False
        self.last_img_time = self.get_clock().now()

        if self.detector is not None:
            results = self.detector.detect(
                frame, estimate_tag_pose=False,
                camera_params=(self.fx, self.fy, self.cx, self.cy),
                tag_size=self.tag_size)
            # 按 target_tag_id 过滤; -1 表示取第一个
            chosen = None
            for r in results:
                if self.target_tag_id < 0 or r.tag_id == self.target_tag_id:
                    chosen = r
                    break
            if chosen is not None:
                self.tag_detected = True
                self.tag_pixel = (float(chosen.center[0]),
                                  float(chosen.center[1]))
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
        now = self.get_clock().now()

        # ---- 超时保护 ----
        img_age = (now - self.last_img_time).nanoseconds / 1e9
        hgt_age = (now - self.last_height_time).nanoseconds / 1e9
        if img_age > self.img_timeout or hgt_age > self.height_timeout:
            if self.state != ST_SEARCH and self.state != ST_LANDED:
                self.state = ST_SEARCH
                self.get_logger().warn(
                    f"传感器超时 (img={img_age:.1f}s, hgt={hgt_age:.1f}s), "
                    "回 SEARCH 悬停")
            self._publish(*self.current_target)
            self.state_pub.publish(String(data=self.state))
            return

        # 未收到位姿时只悬停, 不发布跳变目标
        if not self.pose_received:
            self._publish(0.0, 0.0, self.hover_height)
            self.state_pub.publish(String(data=self.state))
            return

        # 像素偏差 (考虑相机安装方向)
        dx_px = self.tag_pixel[0] - self.cx
        dy_px = self.tag_pixel[1] - self.cy
        if self.flip_x:
            dx_px = -dx_px
        if self.flip_y:
            dy_px = -dy_px

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
                gx, gy = pixel_to_ground(dx_px, dy_px, h, self.fx, self.fy)
                tx = self.current_target[0] + gx
                ty = self.current_target[1] + gy
                self.current_target = (tx, ty, self.hover_height)
                self._publish(tx, ty, self.hover_height)
                if abs(dx_px) < self.align_tol and abs(dy_px) < self.align_tol:
                    self.state = ST_DESCEND
                    self.get_logger().info("对齐完成, 开始降高")

        elif self.state == ST_DESCEND:
            if not self.tag_detected:
                self.state = ST_ALIGN
            else:
                gx, gy = pixel_to_ground(dx_px, dy_px, h, self.fx, self.fy)
                tx = self.current_target[0] + gx * 0.5
                ty = self.current_target[1] + gy * 0.5
                # 只有当前高度接近目标高度才继续降, 避免连续下压
                cur_z = self.current_target[2]
                if self.height >= cur_z - 0.15:
                    new_z = max(cur_z - self.descend_step, self.landed_h)
                else:
                    new_z = cur_z
                self.current_target = (tx, ty, new_z)
                self._publish(tx, ty, new_z)
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
