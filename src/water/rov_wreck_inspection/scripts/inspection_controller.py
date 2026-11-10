#!/usr/bin/env python3
"""沉船环绕巡检与定点采样控制节点。

控制流程：

1. **环绕巡检**：ROV 沿沉船外侧椭圆轨迹运动，机头始终朝向沉船中心，
   保证船体（以及船体侧面的结构标记板）持续处于相机视野内。
2. **视觉引导**：订阅视觉检测结果，用标记板在像平面上的横向偏移量修正机头偏航，
   使标记板回到画面中心；检测结果超时未更新则回退到纯几何朝向。
3. **定点采样**：到达预设采样点时停止运动并悬停 3.5 s，期间用视觉判定标记板
   是否位于画面中心且测距在 1.5～6.0 m 内；确认成功则该采样点标记为绿色，
   超时未确认则标记为橙色（视觉弱）。
4. **可视化**：发布规划轨迹 `/inspection/planned_path`、实际轨迹
   `/inspection/trajectory`、采样点 MarkerArray `/inspection/sample_points`。
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_to_quat(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class InspectionController(Node):
    def __init__(self):
        super().__init__("inspection_controller")

        default_cfg = os.path.join(
            get_package_share_directory("rov_wreck_inspection"),
            "config", "inspection.yaml")
        cfg_path = self.declare_parameter("config_file", default_cfg).value
        cfg = None
        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
        except OSError:
            self.get_logger().warn(
                f"无法读取配置文件 {cfg_path}，使用内置巡检参数")
        cfg = cfg or {}

        self.cx, self.cy = [float(v) for v in cfg.get("wreck_center", [0.0, 0.0])]
        self.a = float(cfg.get("orbit_a", 6.6))
        self.b = float(cfg.get("orbit_b", 3.6))
        self.z = float(cfg.get("orbit_z", -3.6))
        self.speed = float(cfg.get("speed", 0.45))
        self.hold_time = float(cfg.get("hold_time", 3.5))
        self.confirm_time = float(cfg.get("confirm_time", 0.6))
        self.hfov = float(cfg.get("camera_hfov", 1.22))
        self.loop = bool(cfg.get("loop", True))
        self.samples = cfg.get("sample_points", [
            {"name": "S1", "t": 0.559}, {"name": "S2", "t": 2.583},
            {"name": "S3", "t": 3.700}, {"name": "S4", "t": 5.724}])
        self.t = float(cfg.get("start_t", 0.0))

        self.x, self.y = self.point_at(self.t)
        self.yaw = self.yaw_to_center(self.x, self.y)
        self.state = "CRUISE"
        self.idx = 0
        self.hold_start = 0.0
        self.confirm_since = None
        self.vision_confirmed = False
        self.sample_status = {s["name"]: 0 for s in self.samples}  # 0待检 1已确认 2视觉弱
        self.last_vision = None
        self.last_vision_time = 0.0
        self.lost_count = 0

        self.traj = []
        self.last_traj_time = 0.0
        self.last_marker_time = 0.0
        self.last_plan_time = 0.0
        self.last_tick_time = None

        self.path_pub = self.create_publisher(Path, "/inspection/trajectory", 10)
        self.plan_pub = self.create_publisher(Path, "/inspection/planned_path", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/inspection/sample_points", 10)
        self.status_pub = self.create_publisher(String, "/inspection/status", 10)
        self.event_pub = self.create_publisher(
            String, "/inspection/sample_event", 10)
        self.odom_pub = self.create_publisher(Odometry, "/rov/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Float32MultiArray, "/rov/hull_detection", self.on_vision, 10)

        self.set_state_cli = self.create_client(
            SetEntityState, "/gazebo/set_entity_state")
        self.get_logger().info("等待 /gazebo/set_entity_state 服务 ...")
        self.set_state_cli.wait_for_service(timeout_sec=15.0)

        self.timer = self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            f"巡检控制启动：椭圆 {self.a}x{self.b} m，速度 {self.speed} m/s，"
            f"{len(self.samples)} 个采样点")

    # ------------------------------------------------------------------
    def now_sec(self):
        """统一使用节点时钟（仿真时间）计时，避免与 Gazebo 时钟混用。"""
        return self.get_clock().now().nanoseconds / 1e9

    def point_at(self, t):
        return (self.cx + self.a * math.cos(t), self.cy + self.b * math.sin(t))

    def yaw_to_center(self, x, y):
        return math.atan2(self.cy - y, self.cx - x)

    def ellipse_rate(self, t):
        """椭圆参数角变化率：|dP/dt| = sqrt(a^2 sin^2 t + b^2 cos^2 t)。"""
        return math.sqrt((self.a * math.sin(t)) ** 2 +
                         (self.b * math.cos(t)) ** 2)

    # ------------------------------------------------------------------
    def on_vision(self, msg):
        data = list(msg.data)
        if len(data) < 10:
            return
        self.last_vision = {
            "hull": data[0] > 0.5,
            "marker": data[4] > 0.5,
            "u": data[5],
            "dist": data[7],
        }
        self.last_vision_time = self.now_sec()

    def vision_of(self, seg):
        """取视觉量，超过 1.5 s 未更新的结果视为失效。"""
        if self.last_vision is None:
            return None
        if self.now_sec() - self.last_vision_time > 1.5:
            return None
        return self.last_vision.get(seg)

    # ------------------------------------------------------------------
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

    def publish_tf_odom(self):
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
        now = self.now_sec()
        # 用两次回调的仿真时钟差推进参数角，掉帧或暂停恢复时不会漂移
        dt = 0.05 if self.last_tick_time is None else now - self.last_tick_time
        self.last_tick_time = now
        dt = max(0.01, min(dt, 0.2))

        if self.state == "CRUISE":
            self.t += self.speed * dt / max(self.ellipse_rate(self.t), 1e-3)

            if self.idx < len(self.samples) and self.t >= float(self.samples[self.idx]["t"]):
                self.t = float(self.samples[self.idx]["t"])
                self.x, self.y = self.point_at(self.t)
                self.state = "SAMPLE"
                self.hold_start = now
                self.confirm_since = None
                self.vision_confirmed = False
                self.get_logger().info(
                    f"到达采样点 {self.samples[self.idx]['name']}，悬停采样 "
                    f"({self.hold_time:.1f} s)")

            if self.t >= 2.0 * math.pi:
                self.t -= 2.0 * math.pi
                self.idx = 0
                self.sample_status = {s["name"]: 0 for s in self.samples}
                self.traj = []
                self.get_logger().info("完成一圈环绕巡检，开始新一轮")

            self.x, self.y = self.point_at(self.t)
            yaw_des = self.yaw_to_center(self.x, self.y)
            marker = self.vision_of("marker")
            if marker:
                # 标记板相对光轴的偏角（右为正），反向补偿到机头朝向
                err = math.atan(2.0 * self.last_vision["u"] * math.tan(self.hfov / 2.0))
                yaw_des -= 1.0 * err
                self.lost_count = 0
            else:
                self.lost_count += 1

        elif self.state == "SAMPLE":
            marker = self.vision_of("marker")
            if marker:
                err = math.atan(2.0 * self.last_vision["u"] * math.tan(self.hfov / 2.0))
                yaw_des = self.yaw - 0.8 * err
                if abs(self.last_vision["u"]) < 0.16 and 1.5 < self.last_vision["dist"] < 6.0:
                    if self.confirm_since is None:
                        self.confirm_since = now
                    elif now - self.confirm_since > self.confirm_time:
                        self.vision_confirmed = True
                else:
                    self.confirm_since = None
            else:
                yaw_des = self.yaw
                self.confirm_since = None

            if now - self.hold_start >= self.hold_time:
                name = self.samples[self.idx]["name"]
                if self.vision_confirmed:
                    self.sample_status[name] = 1
                    text = f"{name} 采样完成，视觉对准，单目估计距离 {self.last_vision['dist']:.2f} m"
                else:
                    self.sample_status[name] = 2
                    text = f"{name} 采样完成，视觉确认超时（降级记录）"
                self.event_pub.publish(String(data=text))
                self.get_logger().info(text)
                self.idx += 1
                self.state = "CRUISE"
                if self.idx >= len(self.samples) and not self.loop:
                    self.state = "DONE"
                    self.get_logger().info("全部采样点完成，巡检结束")
        else:  # DONE
            yaw_des = self.yaw_to_center(self.x, self.y)

        # 偏航角速率限制，避免机头突变
        max_step = 1.2 * dt
        self.yaw = wrap_angle(self.yaw + max(-max_step, min(max_step, wrap_angle(yaw_des - self.yaw))))

        self.apply_pose()
        self.publish_tf_odom()

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
            path.header.stamp = Time()
            path.poses = list(self.traj)
            self.path_pub.publish(path)

        if now - self.last_plan_time > 2.0:
            self.last_plan_time = now
            self.publish_planned_path()

        if now - self.last_marker_time > 0.5:
            self.last_marker_time = now
            self.publish_markers()

        self.publish_status()

    # ------------------------------------------------------------------
    def publish_planned_path(self):
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = Time()
        n = 180
        for i in range(n + 1):
            t = 2.0 * math.pi * i / n
            x, y = self.point_at(t)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = self.z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.plan_pub.publish(path)

    def publish_markers(self):
        arr = MarkerArray()
        # 时间戳置零：RViz 在软件渲染下处理带时间戳的 marker 时会因坐标系查找
        # 失败拿到非法包围盒，触发 Ogre AxisAlignedBox 断言；零时间戳表示"最新"。
        stamp = Time()

        for i, sp in enumerate(self.samples):
            t = float(sp["t"])
            x, y = self.point_at(t)
            name = sp["name"]
            status = self.sample_status[name]

            # 采样点用三段正交线段绘制：SPHERE / CUBE 等网格型 marker 在软件渲染
            # 环境下会触发 Ogre AxisAlignedBox 断言，LINE_LIST 渲染稳定。
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = stamp
            m.ns = "sample_points"
            m.id = i
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.05
            if status == 1:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.95, 0.35, 0.95
            elif status == 2:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.62, 0.10, 0.95
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.12, 0.65, 1.0, 0.85
            r = 0.30
            m.points = [
                Point(x=x - r, y=y, z=self.z), Point(x=x + r, y=y, z=self.z),
                Point(x=x, y=y - r, z=self.z), Point(x=x, y=y + r, z=self.z),
                Point(x=x, y=y, z=self.z - r), Point(x=x, y=y, z=self.z + r),
            ]
            arr.markers.append(m)

            # 由采样点指向沉船中心的观测视线（细线，避免 ARROW 在部分 RViz 版本下触发渲染断言）
            arrow = Marker()
            arrow.header.frame_id = "odom"
            arrow.header.stamp = stamp
            arrow.ns = "inspect_rays"
            arrow.id = 50 + i
            arrow.type = Marker.LINE_LIST
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = 0.03
            arrow.color.r, arrow.color.g, arrow.color.b = 0.35, 0.85, 1.0
            arrow.color.a = 0.55
            arrow.points = [
                Point(x=x, y=y, z=self.z),
                Point(x=x + (self.cx - x) * 0.35,
                      y=y + (self.cy - y) * 0.35,
                      z=self.z - 0.9),
            ]
            arr.markers.append(arrow)

            txt = Marker()
            txt.header.frame_id = "odom"
            txt.header.stamp = stamp
            txt.ns = "labels"
            txt.id = 100 + i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x, txt.pose.position.y = x, y
            txt.pose.position.z = self.z + 0.55
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.30
            if status == 1:
                txt.color.r = 0.4
                txt.color.g = 1.0
                txt.color.b = 0.5
            elif status == 2:
                txt.color.r = 1.0
                txt.color.g = 0.7
                txt.color.b = 0.3
            else:
                txt.color.r = 0.6
                txt.color.g = 0.85
                txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = f"{name}" if status == 0 else f"{name} OK" if status == 1 else f"{name} WEAK"
            arr.markers.append(txt)

        # 沉船目标标签
        wreck = Marker()
        wreck.header.frame_id = "odom"
        wreck.header.stamp = stamp
        wreck.ns = "labels"
        wreck.id = 200
        wreck.type = Marker.TEXT_VIEW_FACING
        wreck.action = Marker.ADD
        wreck.pose.position.x = self.cx
        wreck.pose.position.y = self.cy
        wreck.pose.position.z = self.z + 1.6
        wreck.pose.orientation.w = 1.0
        wreck.scale.z = 0.34
        wreck.color.r = 1.0
        wreck.color.g = 0.85
        wreck.color.b = 0.35
        wreck.color.a = 1.0
        wreck.text = "SHIPWRECK TARGET"
        arr.markers.append(wreck)

        # ROV 本体轮廓（LINE_LIST 绘制 12 条棱 + 机头指向线，替代 RobotModel）
        rov = Marker()
        rov.header.frame_id = "odom"
        rov.header.stamp = stamp
        rov.ns = "rov_body"
        rov.id = 300
        rov.type = Marker.LINE_LIST
        rov.action = Marker.ADD
        rov.pose.orientation.w = 1.0
        rov.scale.x = 0.045
        rov.color.r, rov.color.g, rov.color.b, rov.color.a = 0.95, 0.50, 0.10, 1.0

        hx, hy, hz = 0.39, 0.28, 0.18
        cy_, sy_ = math.cos(self.yaw), math.sin(self.yaw)
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    dx, dy, dz = sx * hx, sy * hy, sz * hz
                    corners.append(Point(
                        x=self.x + dx * cy_ - dy * sy_,
                        y=self.y + dx * sy_ + dy * cy_,
                        z=self.z + dz))
        edges = [(0, 1), (0, 2), (1, 3), (2, 3),
                 (4, 5), (4, 6), (5, 7), (6, 7),
                 (0, 4), (1, 5), (2, 6), (3, 7)]
        pts = []
        for a, b in edges:
            pts.append(corners[a])
            pts.append(corners[b])
        # 机头指向
        pts.append(Point(x=self.x + (hx + 0.10) * cy_, y=self.y + (hx + 0.10) * sy_, z=self.z))
        pts.append(Point(x=self.x + (hx + 0.55) * cy_, y=self.y + (hx + 0.55) * sy_, z=self.z))
        rov.points = pts
        arr.markers.append(rov)

        # ROV 状态文字
        status = Marker()
        status.header.frame_id = "odom"
        status.header.stamp = stamp
        status.ns = "labels"
        status.id = 201
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = self.x
        status.pose.position.y = self.y
        status.pose.position.z = self.z + 1.05
        status.pose.orientation.w = 1.0
        status.scale.z = 0.26
        status.color.r = 0.35
        status.color.g = 1.0
        status.color.b = 0.45
        status.color.a = 1.0
        status.text = self.status_text()
        arr.markers.append(status)

        self.marker_pub.publish(arr)

    def status_text(self):
        marker = self.vision_of("marker")
        if self.state == "SAMPLE":
            return f"SAMPLING {self.samples[self.idx]['name']}"
        if self.state == "DONE":
            return "INSPECTION COMPLETE"
        if marker:
            return f"CRUISE {self.last_vision['dist']:.1f} m"
        return "VISION SEARCH"

    def publish_status(self):
        marker = self.vision_of("marker")
        progress = 100.0 * self.t / (2.0 * math.pi)
        text = f"{self.state} {progress:5.1f}%"
        if marker:
            text += f" | marker dist={self.last_vision['dist']:.2f}m u={self.last_vision['u']:+.2f}"
        elif self.lost_count > 20:
            text += " | vision target lost"
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = InspectionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
