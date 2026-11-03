"""ROS 2 节点封装：把 GNSS 帧交给完整性门控，并发布可信位姿与状态。

只在 `python main.py --ros` 时导入。判据逻辑全部在 integrity_math.py，
本文件只做经纬高到本地坐标的换算、话题收发和看门狗，不含判据。
"""

from __future__ import annotations

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Int32, String

from .integrity_math import Config, Fix, Monitor, Output

# WGS84 在小范围内的等距近似，仅用于把经纬度换成本地米制坐标
METERS_PER_DEG_LAT = 110540.0
METERS_PER_DEG_LON = 111320.0


class GnssIntegrityNode(Node):
    """订阅 /drone/gnss，输出 /drone/pose_validated 与有效性标志。"""

    def __init__(self) -> None:
        super().__init__("gnss_integrity_monitor")
        defaults = {
            "max_speed": 12.0,
            "max_vertical_speed": 6.0,
            "position_margin": 1.5,
            "accuracy_limit": 3.0,
            "min_fix_type": 2,
            "min_sats": 8,
            "stale_timeout": 1.5,
            "max_extrapolate_dt": 0.5,
            "suspect_limit": 2,
            "recovery_streak": 5,
            "reacquisition_radius": 3.0,
            "origin_lat": 0.0,
            "origin_lon": 0.0,
            "origin_alt": 0.0,
            "default_accuracy": 0.5,
            "frame_id": "map",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def p(name: str) -> float:
            return self.get_parameter(name).value

        self._cfg = Config(
            max_speed=float(p("max_speed")),
            max_vertical_speed=float(p("max_vertical_speed")),
            position_margin=float(p("position_margin")),
            accuracy_limit=float(p("accuracy_limit")),
            min_fix_type=int(p("min_fix_type")),
            min_sats=int(p("min_sats")),
            stale_timeout=float(p("stale_timeout")),
            max_extrapolate_dt=float(p("max_extrapolate_dt")),
            suspect_limit=int(p("suspect_limit")),
            recovery_streak=int(p("recovery_streak")),
            reacquisition_radius=float(p("reacquisition_radius")),
        )
        self._monitor = Monitor(self._cfg)
        self._satellites: Optional[int] = None

        self._pub_pose = self.create_publisher(
            PoseStamped, "/drone/pose_validated", 10
        )
        self._pub_valid = self.create_publisher(Bool, "/drone/gnss_valid", 10)
        self._pub_status = self.create_publisher(
            String, "/drone/gnss_status", 10
        )

        self.create_subscription(
            NavSatFix, "/drone/gnss", self._on_fix, 10
        )
        # 可选：NavSatFix 没有卫星数字段，若上游能提供则订阅后覆盖默认值
        self.create_subscription(
            Int32, "/drone/gnss_satellites", self._on_satellites, 10
        )
        # 完全收不到消息时也要能进入 HOLD，不能只依赖回调里的间隔判断
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            "GNSS 完整性门控已启动，配置: " + json.dumps(
                {
                    "max_speed": self._cfg.max_speed,
                    "stale_timeout": self._cfg.stale_timeout,
                    "suspect_limit": self._cfg.suspect_limit,
                    "recovery_streak": self._cfg.recovery_streak,
                },
                ensure_ascii=False,
            )
        )

    def _on_satellites(self, msg: Int32) -> None:
        self._satellites = int(msg.data)

    def _local_xyz(self, msg: NavSatFix):
        lat0 = float(self.get_parameter("origin_lat").value)
        lon0 = float(self.get_parameter("origin_lon").value)
        alt0 = float(self.get_parameter("origin_alt").value)
        x = (
            (msg.longitude - lon0)
            * METERS_PER_DEG_LON
            * math.cos(math.radians(lat0))
        )
        y = (msg.latitude - lat0) * METERS_PER_DEG_LAT
        z = msg.altitude - alt0
        return x, y, z

    def _fix_type(self, status: int) -> int:
        if status == NavSatStatus.STATUS_NO_FIX:
            return 0
        if status in (NavSatStatus.STATUS_FIX, NavSatStatus.STATUS_SBAS_FIX):
            return 2
        if status == NavSatStatus.STATUS_GBAS_FIX:
            return 3
        return 0

    def _accuracy(self, msg: NavSatFix) -> float:
        variance = msg.position_covariance[0] if msg.position_covariance else 0.0
        if (
            msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
            or variance <= 0.0
        ):
            return float(self.get_parameter("default_accuracy").value)
        return math.sqrt(float(variance))

    def _timestamp(self, msg: NavSatFix) -> float:
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if t <= 0.0:  # 上游没填时间戳时退回节点时钟
            t = self.get_clock().now().nanoseconds * 1e-9
        return t

    def _on_fix(self, msg: NavSatFix) -> None:
        x, y, z = self._local_xyz(msg)
        fix = Fix(
            t=self._timestamp(msg),
            x=x,
            y=y,
            z=z,
            fix_type=self._fix_type(msg.status.status),
            num_sats=(
                self._satellites
                if self._satellites is not None
                else self._cfg.min_sats
            ),
            accuracy=self._accuracy(msg),
        )
        out = self._monitor.update(fix)
        self._publish(out, msg.header.stamp, msg.header.frame_id)

    def _publish(self, out: Output, stamp, frame_id: str) -> None:
        fid = frame_id or str(self.get_parameter("frame_id").value)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = fid
        pose.pose.position.x = float(out.x)
        pose.pose.position.y = float(out.y)
        pose.pose.position.z = float(out.z)
        pose.pose.orientation.w = 1.0
        self._pub_pose.publish(pose)

        valid = Bool()
        valid.data = bool(out.valid)
        self._pub_valid.publish(valid)

        status = String()
        status.data = json.dumps(
            {
                "state": out.state.value,
                "valid": out.valid,
                "action": out.action,
                "reason": out.reason,
                "consecutive_bad": out.consecutive_bad,
                "consecutive_good": out.consecutive_good,
            },
            ensure_ascii=False,
        )
        self._pub_status.publish(status)

    def _watchdog(self) -> None:
        if self._monitor.last_seen_t is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._monitor.last_seen_t <= self._cfg.stale_timeout:
            return
        out = self._monitor.note_timeout()
        if out is not None:
            self.get_logger().warn(
                "定位流超时，进入 HOLD：下游应保持悬停，停止使用定位闭环"
            )
            self._publish(out, self.get_clock().now().to_msg(), "")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = GnssIntegrityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
