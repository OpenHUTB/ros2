# -*- coding: utf-8 -*-
"""无人机激光雷达避障的 ROS 2 节点。

本节点是 ``drone_control.py``（纯 CARLA / OpenHUTB Python 脚本）的 ROS 2
包装：把"订阅机载激光雷达点云 → 求最近障碍物距离 → 低于阈值停车、否则
前飞"这条控制律搬进 ROS 2 话题世界，因此可以用 ``ros2 launch`` 启动、
用 ``ros2 topic`` 观测，也能和 ``carla_ros_bridge`` 发布的点云直接对接。

话题
----
订阅 ``lidar_topic``（默认 ``/carla/lidar/point_cloud``，``sensor_msgs/PointCloud2``）
发布 ``cmd_vel_topic``（默认 ``/uav/cmd_vel``，``geometry_msgs/Twist``，
``linear.x`` 为机体系前向速度，其余分量为 0）
可选发布 ``~/min_range``（``std_msgs/Float32``，实时最近障碍距离，便于调试）

安全策略
--------
点云超过 ``data_timeout_s`` 未更新时按"看不见就停"处理（发布零速度），
而不是沿用上一帧的判断继续前飞。
"""

from __future__ import annotations

import math
import struct

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32


def _xyz_offsets(msg: PointCloud2):
    """取出 PointCloud2 里 x / y / z 三个字段的字节偏移，缺字段返回 None。"""
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z") and field.name not in offsets:
            offsets[field.name] = field.offset
    if len(offsets) != 3:
        return None
    return offsets


def min_range_from_cloud(msg: PointCloud2) -> float:
    """返回点云里最近点的欧氏距离（米）；没有有效点时返回 ``inf``。

    逐点用 ``struct`` 解析，不依赖 numpy / sensor_msgs_py，保证本包只依赖
    ROS 2 基础消息包就能跑。CARLA 的 lidar 对没有命中的光束给出 (0, 0, 0)，
    这类点按"无回波"丢弃；NaN / inf 同样跳过。
    """
    offsets = _xyz_offsets(msg)
    if offsets is None or msg.point_step <= 0 or not msg.data:
        return float("inf")

    endian = ">" if msg.is_bigendian else "<"
    step = msg.point_step
    ox, oy, oz = offsets["x"], offsets["y"], offsets["z"]
    contiguous = (ox, oy, oz) == (0, 4, 8)   # CARLA / PCL 的常见布局
    best = float("inf")
    for base in range(0, len(msg.data) - step + 1, step):
        if contiguous:
            x, y, z = struct.unpack_from(endian + "fff", msg.data, base)
        else:
            x = struct.unpack_from(endian + "f", msg.data, base + ox)[0]
            y = struct.unpack_from(endian + "f", msg.data, base + oy)[0]
            z = struct.unpack_from(endian + "f", msg.data, base + oz)[0]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue        # 无回波
        d = math.sqrt(x * x + y * y + z * z)
        if d < best:
            best = d
    return best


class UavLidarObstacleNode(Node):
    """感知-决策-控制闭环：点云最小距离 → 前飞 / 停车。"""

    def __init__(self):
        super().__init__("uav_lidar_obstacle")

        self.declare_parameter("lidar_topic", "/carla/lidar/point_cloud")
        self.declare_parameter("cmd_vel_topic", "/uav/cmd_vel")
        self.declare_parameter("stop_distance", 4.0)
        self.declare_parameter("forward_speed", 2.0)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("data_timeout_s", 1.0)
        self.declare_parameter("publish_min_range", True)

        lidar_topic = self.get_parameter("lidar_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        rate = float(self.get_parameter("control_rate_hz").value)
        self.data_timeout_s = float(self.get_parameter("data_timeout_s").value)
        publish_range = bool(self.get_parameter("publish_min_range").value)

        if rate <= 0.0:
            self.get_logger().warn("control_rate_hz 必须为正，回退到 10 Hz")
            rate = 10.0

        self._min_range = float("inf")
        self._last_cloud_ns = None
        self._last_state = None

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 1)
        self._range_pub = (self.create_publisher(Float32, "~/min_range", 1)
                           if publish_range else None)
        self.create_subscription(PointCloud2, lidar_topic, self._on_cloud,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            "uav_lidar_obstacle 已启动：点云 %s → 速度 %s，停车阈值 %.2f m，"
            "前飞速度 %.2f m/s，控制频率 %.1f Hz"
            % (lidar_topic, cmd_vel_topic, self.stop_distance,
               self.forward_speed, rate))

    # ── 回调 ────────────────────────────────────────────────────────────
    def _on_cloud(self, msg: PointCloud2) -> None:
        self._min_range = min_range_from_cloud(msg)
        self._last_cloud_ns = self.get_clock().now().nanoseconds
        if self._range_pub is not None:
            value = self._min_range
            self._range_pub.publish(
                Float32(data=value if math.isfinite(value) else float("nan")))

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_cloud_ns is None:
            self._publish_stop("尚未收到点云")
            return
        age = (now_ns - self._last_cloud_ns) / 1e9
        if age > self.data_timeout_s:
            self._publish_stop("点云已 %.2f s 未更新" % age)
            return
        if self._min_range < self.stop_distance:
            self._publish_speed(0.0,
                                "前方障碍物 %.2f m，悬停" % self._min_range)
        else:
            self._publish_speed(self.forward_speed,
                                "前方安全 %.2f m，向前飞行" % self._min_range)

    # ── 工具 ────────────────────────────────────────────────────────────
    def _publish_speed(self, speed: float, reason: str) -> None:
        state = "stop" if speed == 0.0 else "move"
        if state != self._last_state:
            self.get_logger().info(reason)
            self._last_state = state
        msg = Twist()
        msg.linear.x = speed
        self._cmd_pub.publish(msg)

    def _publish_stop(self, reason: str) -> None:
        if self._last_state != "stop":
            self.get_logger().warn("%s，按故障安全策略停车" % reason)
            self._last_state = "stop"
        self._cmd_pub.publish(Twist())

    def publish_stop(self) -> None:
        """显式发布一次零速度（退出清理用）。"""
        self._cmd_pub.publish(Twist())


def main(args=None) -> int:
    rclpy.init(args=args)
    node = UavLidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
