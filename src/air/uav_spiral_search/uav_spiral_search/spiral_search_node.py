"""Spiral search mission node: HSV detection + spiral tracking on /openhutb topics."""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .spiral_search import Settings as SearchSettings, SpiralSearchController
from .target_detector import Settings as DetectorSettings, detect_target


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class SpiralSearchNode(Node):
    def __init__(self, **kwargs):
        super().__init__('spiral_search', **kwargs)
        search_defaults = vars(SearchSettings())
        for name, value in search_defaults.items():
            self.declare_parameter(name, value)
        for name, value in vars(DetectorSettings()).items():
            self.declare_parameter('det_' + name, value)
        for name, value in {
            'dry_run': True,
            'control_dt': 0.05,
            'show_window': True,
            'image_topic': '/openhutb/camera/rgb',
            'odom_topic': '/openhutb/odom',
            'cmd_vel_topic': '/openhutb/cmd_vel',
            'preview_topic': '/openhutb/spiral_search/preview_cmd_vel',
            'status_topic': '/openhutb/spiral_search/status',
        }.items():
            self.declare_parameter(name, value)

        self.controller = SpiralSearchController(SearchSettings(**{
            name: self.get_parameter(name).value for name in search_defaults}))
        self.detector = DetectorSettings(**{
            name[len('det_'):]: self.get_parameter('det_' + name).value
            for name in vars(DetectorSettings())})

        self.dry_run = bool(self.get_parameter('dry_run').value)
        dt = float(self.get_parameter('control_dt').value)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('control_dt must be finite and positive')
        self.show_window = bool(self.get_parameter('show_window').value)

        self.last_odom_stamp = -1
        self.last_status = None
        self.last_detection = None

        topic = lambda name: str(self.get_parameter(name).value)
        self.cmd_pub = self.create_publisher(
            Twist, topic('preview_topic') if self.dry_run else topic('cmd_vel_topic'), 1)
        self.status_pub = self.create_publisher(String, topic('status_topic'), 1)
        self.subscriptions_ = [
            self.create_subscription(Image, topic('image_topic'),
                                     self.image_callback, qos_profile_sensor_data),
            self.create_subscription(Odometry, topic('odom_topic'),
                                     self.odom_callback, qos_profile_sensor_data),
        ]
        self.timer = self.create_timer(dt, self.control_step)
        self.get_logger().info(
            f'Spiral search ready; dry_run={self.dry_run}, '
            f'hue=[{self.detector.hue_min}, {self.detector.hue_max}]')

    # ------------------------------------------------------------------
    # 里程计：与视觉居中模式相同的校验（坐标系、四元数、姿态、时序）
    # ------------------------------------------------------------------
    def fresh(self, stamp, timeout):
        age = (self.get_clock().now().nanoseconds - stamp_ns(stamp)) / 1e9
        return stamp_ns(stamp) > 0 and 0 <= age <= timeout

    def odom_callback(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if not self.fresh(msg.header.stamp,
                          self.controller.settings.odom_timeout):
            self.controller.pose = None
            return
        if stamp <= self.last_odom_stamp:
            self.controller.pose = None
            return
        self.last_odom_stamp = stamp
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        values = (q.x, q.y, q.z, q.w, p.x, p.y, p.z)
        norm = math.sqrt(sum(v * v for v in values[:4]))
        if (msg.header.frame_id != 'airsim_ned'
                or not all(math.isfinite(v) for v in values)
                or abs(norm - 1) > 0.05):
            self.controller.pose = None
            return
        x, y, z, w = [v / norm for v in values[:4]]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        if max(abs(roll), abs(pitch)) > math.radians(15):
            self.controller.pose = None
            return
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        self.controller.update_pose(p.x, p.y, p.z, yaw,
                                    time.monotonic() - age)

    # ------------------------------------------------------------------
    # 图像：HSV 检测 + 可选调试窗口
    # ------------------------------------------------------------------
    def image_callback(self, msg):
        visible = False
        detection = None
        if (msg.width > 0 and msg.height > 0
                and msg.encoding in ('bgr8', 'rgb8')):
            frame = np.frombuffer(msg.data, dtype=np.uint8)
            if frame.size == msg.height * msg.width * 3:
                frame = frame.reshape(msg.height, msg.width, 3)
                if msg.encoding == 'rgb8':
                    frame = frame[:, :, ::-1]
                detection = detect_target(frame, self.detector)
                self.last_detection = detection
                visible = detection is not None
                if self.show_window:
                    self._show(frame, detection)
        # 检测时刻按图像采集时间戳对齐，延迟到达的图像不额外获得超时窗口。
        age = (self.get_clock().now().nanoseconds
               - stamp_ns(msg.header.stamp)) / 1e9
        now = time.monotonic() - age if msg.header.stamp.sec > 0 \
            else time.monotonic()
        self.controller.update_detection(
            visible, now, detection.cx if detection is not None else 0.0)

    def _show(self, frame, detection):
        canvas = frame.copy()
        h, w = canvas.shape[:2]
        state = self.controller.state
        text = (f'state={state} '
                f'confirm={self.controller.confirm_count}')
        cv2.putText(canvas, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(canvas, (w // 2, h // 2), 6, (0, 255, 255), 1)
        if detection is not None:
            u = int((detection.cx + 1.0) * 0.5 * w)
            v = int((detection.cy + 1.0) * 0.5 * h)
            cv2.line(canvas, (u - 14, v), (u + 14, v), (0, 0, 255), 2)
            cv2.line(canvas, (u, v - 14), (u, v + 14), (0, 0, 255), 2)
            cv2.putText(canvas, f'target {detection.area_fraction:.3f}',
                        (u + 18, v), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2, cv2.LINE_AA)
        try:
            cv2.imshow('spiral_search', canvas)
            cv2.waitKey(1)
        except cv2.error:
            # 无显示环境（纯命令行/无 X server）时自动关闭窗口功能。
            self.show_window = False
            self.get_logger().warning(
                'Cannot open CV window; continuing without preview.')

    # ------------------------------------------------------------------
    # 控制循环
    # ------------------------------------------------------------------
    def control_step(self):
        command, state, report = self.controller.step(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = command
        self.cmd_pub.publish(msg)
        report['dry_run'] = self.dry_run
        status = String()
        status.data = json.dumps(report)
        self.status_pub.publish(status)
        if state != self.last_status:
            if state == 'locked':
                self.get_logger().warning(
                    f'Target locked after {report["search_time_s"]} s, '
                    f'path {report["path_len_m"]} m; '
                    f'estimate NED {report["target_estimate_ned"]}. '
                    'Hovering; land the drone manually to finish.')
            elif state == 'exhausted':
                self.get_logger().warning(
                    f'Search area exhausted at radius '
                    f'{report["radius_m"]} m without a confirmed target; '
                    'hovering.')
            else:
                self.get_logger().info(f'Spiral search: {state}')
            self.last_status = state

    def destroy_node(self):
        if rclpy.ok(context=self.context):
            self.cmd_pub.publish(Twist())
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpiralSearchNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
