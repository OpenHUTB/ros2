#!/usr/bin/env python3
"""ROV 定点巡航控制节点。

根据 YAML 航点配置，依次飞向每个 AprilTag 标牌前的悬停点；
当相机检测到目标标牌后，根据视觉检测结果平滑调整机头朝向并对准标牌。
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, String
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

import yaml


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class CruiseController(Node):
    def __init__(self):
        super().__init__("cruise_controller")

        default_cfg = os.path.join(
            get_package_share_directory("rov_apriltag_cruise"), "config", "cruise.yaml")
        cfg_path = self.declare_parameter("config_file", default_cfg).value
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        self.waypoints = cfg["waypoints"]
        self.speed = float(cfg.get("max_speed", 0.55))
        self.hold_time = float(cfg.get("hold_time", 2.5))
        self.detection_range = float(cfg.get("detection_range", 4.5))
        self.heading_rate = float(cfg.get("heading_rate", 1.1))
        self.tolerance = float(cfg.get("waypoint_tolerance", 0.08))
        self.loop = bool(cfg.get("loop", True))

        self.x, self.y, self.z = [float(v) for v in cfg.get("start_pose", [0.0, 0.0, -2.2])]
        self.yaw = float(cfg.get("start_yaw", 0.0))

        self.index = 0
        self.state = "CRUISE"
        self.hold_start = 0.0
        self.detections = {}          # tag_id -> 距离 (相机坐标系)
        self.started = False
        self.traj = []
        self.last_traj_time = 0.0
        self.last_marker_time = 0.0

        self.path_pub = self.create_publisher(Path, "/rov/trajectory", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/rov/waypoints", 10)
        self.status_pub = self.create_publisher(String, "/rov/status", 10)
        self.odom_pub = self.create_publisher(Odometry, "/rov/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.det_sub = self.create_subscription(
            Float32MultiArray, "/rov/tag_detections", self.on_detections, 10)

        self.set_state_cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.get_logger().info("等待 /gazebo/set_entity_state 服务 ...")
        self.set_state_cli.wait_for_service(timeout_sec=10.0)

        self.timer = self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f"巡航控制节点启动：{len(self.waypoints)} 个航点, 速度 {self.speed} m/s")

    # ------------------------------------------------------------------
    def on_detections(self, msg):
        self.detections = {}
        data = list(msg.data)
        for i in range(0, len(data) - 4, 5):
            tag_id = int(data[i])
            dist = float(data[i + 4])
            self.detections[tag_id] = dist

    def target_tag_visible(self):
        wp = self.waypoints[self.index]
        return wp["tag_id"] in self.detections

    def apply_pose(self):
        req = SetEntityState.Request()
        st = EntityState()
        st.name = "rov"
        st.reference_frame = "world"
        st.pose.position.x = self.x
        st.pose.position.y = self.y
        st.pose.position.z = self.z
        qz, qw = yaw_to_quat(self.yaw)
        st.pose.orientation.z = qz
        st.pose.orientation.w = qw
        req.state = st
        self.set_state_cli.call_async(req)

    def publish_tf_and_odom(self):
        stamp = self.get_clock().now().to_msg()
        qz, qw = yaw_to_quat(self.yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

    # ------------------------------------------------------------------
    def on_timer(self):
        now = time.time()
        if not self.started:
            self.apply_pose()
            self.started = True
            return

        wp = self.waypoints[self.index]
        gx, gy, gz = [float(v) for v in wp["goal"]]

        if self.state == "CRUISE":
            dx, dy, dz = gx - self.x, gy - self.y, gz - self.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            # 目标朝向：默认朝航点方向；目标标牌进入视野后改为对准标牌中心
            yaw_des = math.atan2(dy, dx) if dist > 1e-3 else self.yaw
            if self.target_tag_visible() and self.detections[wp["tag_id"]] < self.detection_range:
                tx, ty, _ = [float(v) for v in wp["tag"]]
                yaw_des = math.atan2(ty - self.y, tx - self.x)

            dyaw = wrap_angle(yaw_des - self.yaw)
            max_step = self.heading_rate * 0.05
            self.yaw = wrap_angle(self.yaw + max(-max_step, min(max_step, dyaw)))

            v = min(self.speed, max(0.18, 0.55 * dist))
            step = v * 0.05
            if dist <= step or dist < self.tolerance:
                self.x, self.y, self.z = gx, gy, gz
                self.state = "HOLD"
                self.hold_start = now
                self.get_logger().info(f"到达航点 {wp['name']}，开始视觉对准悬停")
            else:
                self.x += dx / dist * step
                self.y += dy / dist * step
                self.z += dz / dist * step

        elif self.state == "HOLD":
            # 悬停期间继续根据视觉检测微调朝向，保持正对标牌
            tx, ty, tz = [float(v) for v in wp["tag"]]
            yaw_des = math.atan2(ty - self.y, tx - self.x)
            dyaw = wrap_angle(yaw_des - self.yaw)
            max_step = self.heading_rate * 0.05
            self.yaw = wrap_angle(self.yaw + max(-max_step, min(max_step, dyaw)))
            if now - self.hold_start > self.hold_time:
                self.index += 1
                if self.index >= len(self.waypoints):
                    if self.loop:
                        self.index = 0
                    else:
                        self.state = "DONE"
                        self.get_logger().info("全部航点巡航完成")
                        return
                self.state = "CRUISE"
                self.get_logger().info(f"前往航点 {self.waypoints[self.index]['name']}")

        self.apply_pose()
        self.publish_tf_and_odom()

        if now - self.last_traj_time > 0.2:
            self.last_traj_time = now
            ps = PoseStamped()
            ps.header.frame_id = "odom"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = self.x
            ps.pose.position.y = self.y
            ps.pose.position.z = self.z
            qz, qw = yaw_to_quat(self.yaw)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            self.traj.append(ps)
            path = Path()
            path.header.frame_id = "odom"
            path.header.stamp = ps.header.stamp
            path.poses = self.traj
            self.path_pub.publish(path)

        if now - self.last_marker_time > 0.5:
            self.last_marker_time = now
            self.publish_markers()

    # ------------------------------------------------------------------
    def segment(self, arr, marker_id, p1, p2, color, width=0.035):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "frames"
        m.id = marker_id
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], color[3]
        m.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))
        m.points.append(Point(x=float(p2[0]), y=float(p2[1]), z=float(p2[2])))
        arr.markers.append(m)

    def publish_markers(self):
        """只用 LINE_LIST / LINE_STRIP 绘制坐标系、航点与 ROV 轮廓。

        虚拟机软件渲染下 RViz2 的网格类显示（球/立方体/文字）会触发 Ogre 崩溃，
        线类图元不受影响，因此这里统一用线段绘制。
        """
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, wp in enumerate(self.waypoints):
            tx, ty, tz = [float(v) for v in wp["tag"]]
            nx, ny = tx, ty
            norm = math.hypot(nx, ny)
            nx, ny = -nx / norm, -ny / norm     # 标牌法线指向场景中心
            gx, gy, gz = [float(v) for v in wp["goal"]]

            # 标牌坐标系：红=法线方向, 绿=切向, 蓝=竖直向上
            self.segment(arr, 300 + i * 3, (tx, ty, tz), (tx + nx * 0.45, ty + ny * 0.45, tz),
                         (1.0, 0.15, 0.15, 1.0), 0.05)
            self.segment(arr, 301 + i * 3, (tx, ty, tz), (tx - ny * 0.45, ty + nx * 0.45, tz),
                         (0.15, 1.0, 0.2, 1.0), 0.05)
            self.segment(arr, 302 + i * 3, (tx, ty, tz), (tx, ty, tz + 0.45),
                         (0.2, 0.5, 1.0, 1.0), 0.05)

            # 航点圆圈（LINE_STRIP）
            ring = Marker()
            ring.header.frame_id = "odom"
            ring.header.stamp = stamp
            ring.ns = "waypoints"
            ring.id = 400 + i
            ring.type = Marker.LINE_STRIP
            ring.action = Marker.ADD
            ring.pose.orientation.w = 1.0
            ring.scale.x = 0.035
            active = (i == self.index)
            if active:
                ring.color.r, ring.color.g, ring.color.b, ring.color.a = 1.0, 0.55, 0.05, 1.0
            else:
                ring.color.r, ring.color.g, ring.color.b, ring.color.a = 0.10, 0.65, 1.0, 0.85
            for k in range(25):
                ang = 2.0 * math.pi * k / 24.0
                ring.points.append(Point(x=gx + 0.22 * math.cos(ang),
                                        y=gy + 0.22 * math.sin(ang), z=gz))
            arr.markers.append(ring)

        # ROV 本体坐标系与轮廓
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        self.segment(arr, 500, (self.x, self.y, self.z),
                     (self.x + cy * 0.5, self.y + sy * 0.5, self.z), (1.0, 0.2, 0.2, 1.0), 0.045)
        self.segment(arr, 501, (self.x, self.y, self.z),
                     (self.x - sy * 0.4, self.y + cy * 0.4, self.z), (0.2, 1.0, 0.3, 1.0), 0.045)
        self.segment(arr, 502, (self.x, self.y, self.z),
                     (self.x, self.y, self.z + 0.35), (0.25, 0.55, 1.0, 1.0), 0.045)

        # ROV 机身线框（8 个角点，12 条边）
        hx, hy, hz = 0.39, 0.28, 0.18
        corners = []
        for sz in (-1, 1):
            for sx in (-1, 1):
                for sy_ in (-1, 1):
                    corners.append((self.x + cy * sx * hx - sy * sy_ * hy,
                                    self.y + sy * sx * hx + cy * sy_ * hy,
                                    self.z + sz * hz))
        edges = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6),
                 (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
        for k, (a, b) in enumerate(edges):
            self.segment(arr, 600 + k, corners[a], corners[b], (0.55, 0.85, 1.0, 0.9), 0.02)

        self.marker_pub.publish(arr)

        if self.state == "HOLD":
            text = "VISUAL ALIGN - " + self.waypoints[self.index]["name"]
        elif self.state == "DONE":
            text = "CRUISE COMPLETE"
        else:
            text = "CRUISE -> " + self.waypoints[self.index]["name"]
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = CruiseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
