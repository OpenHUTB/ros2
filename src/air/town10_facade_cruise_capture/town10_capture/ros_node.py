"""ROS 2 发布相机、ENU里程计、速度指令及任务状态。"""

import json
import signal
from contextlib import suppress
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .config import CruiseConfig
from .mission import CruiseMission


class CruiseNode(Node):
    def __init__(self):
        super().__init__("town10_facade_cruise_capture")
        defaults = {
            "backend": "mujoco",
            "style": "brick",
            "output": "/tmp/town10_cruise",
            "duration": 100.0,
            "distance": 4.0,
            "speed": 0.8,
            "rate": 20.0,
            "capture_rate": 2.0,
            "host": "127.0.0.1",
            "port": 41451,
            "vehicle": "",
            "camera": "0",
            "route": "relative",
            "takeoff": False,
            "gif": False,
            "waypoint_timeout": 35.0,
            "route_offset": [0.0, 0.0, 0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        config = CruiseConfig(
            **{name: self.get_parameter(name).value for name in defaults}
        )
        config.output = Path(config.output)
        self.image_pub = self.create_publisher(Image, "/town10/image_raw", 10)
        self.odom_pub = self.create_publisher(Odometry, "/town10/odom", 10)
        self.cmd_pub = self.create_publisher(Twist, "/town10/cmd_vel", 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/town10/status", qos)
        self.mission = CruiseMission(config, self._publish)
        self.timer = self.create_timer(1 / config.rate, self._tick)
        self._status("running")

    def _status(self, status):
        message = String()
        message.data = json.dumps(
            {
                "state": status,
                "waypoint": self.mission.waypoint,
                "total": len(self.mission.route),
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(message)

    def _publish(self, sample):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "town10_base_link"
        (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
        ) = map(float, sample["position"])
        q = odom.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, sample["orientation_xyzw"])
        # Odometry twist应在child frame表达；此节点将ENU速度逆旋转到FLU机体。
        from .airsim_backend import quaternion_matrix

        body_velocity = (
            quaternion_matrix(sample["orientation_xyzw"]).T @ sample["velocity"]
        )
        (
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
        ) = map(float, body_velocity)
        self.odom_pub.publish(odom)
        command = Twist()
        command.linear.x, command.linear.y, command.linear.z = map(
            float, sample["command"]
        )
        self.cmd_pub.publish(command)
        if sample["image"] is not None:
            pixels = sample["image"]
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = "town10_camera_optical"
            image.height, image.width = pixels.shape[:2]
            image.encoding = "rgb8"
            image.is_bigendian = 0
            image.step = image.width * 3
            image.data = pixels.tobytes()
            self.image_pub.publish(image)
        self._status("running")

    def _tick(self):
        try:
            running = self.mission.tick()
        except Exception as exc:  # noqa: BLE001 -- RPC/渲染异常均需停止任务并发布终态。
            self.get_logger().error(str(exc))
            running = False
        if not running:
            self.timer.cancel()
            self.cmd_pub.publish(Twist())
            self._status(self.mission.reason)

    def destroy_node(self):
        if not self.mission.finished:
            self.mission.finish()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CruiseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 清理阶段忽略重复SIGINT，保证日志写入和后端悬停不会被第二次中断打断。
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            with suppress(KeyboardInterrupt, ExternalShutdownException):
                if node:
                    node.destroy_node()
            with suppress(KeyboardInterrupt, ExternalShutdownException):
                rclpy.try_shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_handler)
