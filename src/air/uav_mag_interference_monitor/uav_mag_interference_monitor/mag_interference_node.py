"""ROS 2 节点封装：把磁场与 GNSS 观测交给航向门控，并发布推荐航向。

只在 `python main.py --ros` 时导入。判据逻辑全部在
mag_interference_math.py，本文件只做单位换算、航迹向解算、话题收发和
看门狗，不含判据。

话题约定：
  订阅 /drone/magnetic_field  sensor_msgs/MagneticField（特斯拉，内部转 µT）
  订阅 /drone/gnss            sensor_msgs/NavSatFix（用于解算地速与航迹向）
  发布 /drone/heading         std_msgs/Float64（推荐航向，rad，磁北基准）
  发布 /drone/heading_valid   std_msgs/Bool
  发布 /drone/mag_status      std_msgs/String（JSON：状态、来源、判据明细）
"""

from __future__ import annotations

import json
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, NavSatFix
from std_msgs.msg import Bool, Float64, String

from .mag_interference_math import (
    Config,
    FlightSample,
    HeadingMonitor,
    HeadingOutput,
    MagSample,
)

METERS_PER_DEG_LAT = 110540.0
METERS_PER_DEG_LON = 111320.0


class MagInterferenceNode(Node):
    """订阅磁场与 GNSS，输出经过一致性门控的推荐航向。"""

    def __init__(self, **kwargs) -> None:
        # **kwargs 透传给 rclpy.Node（如 parameter_overrides），便于测试与
        # launch 文件注入参数，不改变默认行为。
        super().__init__("mag_interference_monitor", **kwargs)
        defaults = {
            "bias_x": 0.0,
            "bias_y": 0.0,
            "expected_radius": 0.0,     # 起飞前自检得到的水平半径；0 表示未提供
            "declination_deg": 0.0,     # 磁偏角：把磁航向换算到真北
            "min_ground_speed": 2.0,
            "max_heading_error_deg": 15.0,
            "critical_heading_error_deg": 30.0,
            "norm_tolerance_pct": 15.0,
            "degraded_limit": 3,
            "unreliable_limit": 8,
            "recovery_streak": 5,
            "stale_timeout": 1.0,
            "frame_id": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def p(name: str) -> float:
            return self.get_parameter(name).value

        cfg = Config(
            min_ground_speed=float(p("min_ground_speed")),
            max_heading_error_deg=float(p("max_heading_error_deg")),
            critical_heading_error_deg=float(p("critical_heading_error_deg")),
            norm_tolerance_pct=float(p("norm_tolerance_pct")),
            degraded_limit=int(p("degraded_limit")),
            unreliable_limit=int(p("unreliable_limit")),
            recovery_streak=int(p("recovery_streak")),
            stale_timeout=float(p("stale_timeout")),
        )
        radius = float(p("expected_radius"))
        self._monitor = HeadingMonitor(
            (float(p("bias_x")), float(p("bias_y"))),
            cfg,
            expected_radius=radius if radius > 0.0 else None,
        )
        self._declination = math.radians(float(p("declination_deg")))
        self._min_speed = cfg.min_ground_speed

        self._latest_mag: Optional[MagSample] = None
        self._last_fix: Optional[Tuple[float, float, float]] = None
        self._course: Optional[float] = None
        self._speed = 0.0

        self._pub_heading = self.create_publisher(Float64, "/drone/heading", 10)
        self._pub_valid = self.create_publisher(Bool, "/drone/heading_valid", 10)
        self._pub_status = self.create_publisher(String, "/drone/mag_status", 10)

        self.create_subscription(
            MagneticField, "/drone/magnetic_field", self._on_mag, 10
        )
        self.create_subscription(NavSatFix, "/drone/gnss", self._on_fix, 10)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            "磁力计干扰监测已启动，配置: " + json.dumps(
                {
                    "bias_x": self._monitor.bias[0],
                    "bias_y": self._monitor.bias[1],
                    "expected_radius": self._monitor.cal_radius,
                    "min_ground_speed": self._min_speed,
                },
                ensure_ascii=False,
            )
        )

    # --- 订阅回调 -------------------------------------------------------

    def _stamp(self, msg) -> float:
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        return t

    def _on_mag(self, msg: MagneticField) -> None:
        # ROS 用特斯拉，判据层用微特斯拉
        self._latest_mag = MagSample(
            t=self._stamp(msg),
            mx=msg.magnetic_field.x * 1e6,
            my=msg.magnetic_field.y * 1e6,
            mz=msg.magnetic_field.z * 1e6,
        )
        if self._last_fix is None:
            return  # 还没有 GNSS，无法互检，先缓存磁场
        self._update_from_cache(self._stamp(msg))

    def _on_fix(self, msg: NavSatFix) -> None:
        t = self._stamp(msg)
        lat, lon = float(msg.latitude), float(msg.longitude)
        if self._last_fix is not None:
            t0, lat0, lon0 = self._last_fix
            dt = t - t0
            if dt > 1e-3:
                east = (lon - lon0) * METERS_PER_DEG_LON * math.cos(
                    math.radians(lat)
                )
                north = (lat - lat0) * METERS_PER_DEG_LAT
                self._speed = math.hypot(east, north) / dt
                if self._speed >= self._min_speed:
                    self._course = math.atan2(east, north)
                else:
                    self._course = None
        self._last_fix = (t, lat, lon)
        if self._latest_mag is not None:
            self._update_from_cache(t)

    # --- 判据与发布 -----------------------------------------------------

    def _update_from_cache(self, t: float) -> None:
        mag = self._latest_mag
        if mag is None:
            return
        # 磁场缓存已过期时不再用旧数据冒充新观测：观测时间以磁力计自身的
        # 时间戳为准（而不是触发本次回调的 GNSS 时间），否则 GNSS 正常流动
        # 会不断刷新看门狗的 last_seen_t，把真正的磁场话题中断掩盖掉。
        # 过期后的降级交给看门狗 note_timeout 统一处理。
        if t - mag.t > self._monitor.cfg.stale_timeout:
            return
        sample = FlightSample(
            t=mag.t,
            mx=mag.mx,
            my=mag.my,
            mz=mag.mz,
            speed=self._speed,
            course=self._course,
        )
        self._publish(self._monitor.update(sample))

    def _publish(self, out: HeadingOutput) -> None:
        # 磁北 → 真北（磁偏角东偏为正）
        heading = Float64()
        heading.data = float(out.heading + self._declination)
        self._pub_heading.publish(heading)

        valid = Bool()
        valid.data = bool(out.valid)
        self._pub_valid.publish(valid)

        status = String()
        status.data = json.dumps(
            {
                "state": out.state.value,
                "source": out.heading_source,
                "valid": out.valid,
                "reason": out.reason,
                "heading_error_deg": (
                    None
                    if not math.isfinite(out.heading_error_deg)
                    else round(out.heading_error_deg, 2)
                ),
                "field_norm_pct": (
                    None
                    if not math.isfinite(out.field_norm_pct)
                    else round(out.field_norm_pct, 2)
                ),
            },
            ensure_ascii=False,
        )
        self._pub_status.publish(status)

    def _watchdog(self) -> None:
        if self._monitor.last_seen_t is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._monitor.last_seen_t <= self._monitor.cfg.stale_timeout:
            return
        out = self._monitor.note_timeout(now, self._course)
        if out is not None:
            if out.state.value == "UNRELIABLE":
                self.get_logger().warn(
                    "磁场数据超时，航向不可信：下游应改用 GNSS 航迹向或原地悬停"
                )
            self._publish(out)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = MagInterferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
