"""ROS 2 advisory node: fuse weather, predict response, publish risk markers."""

from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from .core.aircraft import AircraftParams, AircraftState
from .core.risk import RiskResult, RiskThresholds, assess_risk
from .core.types import Vec3, vadd, vscale


class WeatherRiskNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_weather_risk")
        for name, default in (
            ("assessment_period_s", 0.5),
            ("horizon_s", 10.0),
            ("dt_s", 0.25),
            ("samples", 64),
            ("seed", 42),
            ("max_wind_mps", 11.0),
            ("max_tilt_deg", 33.0),
            ("max_position_error_m", 2.5),
            ("max_control_saturation", 1.10),
            ("turbulence_sigma_mps", 1.3),
            ("turbulence_tau_s", 3.0),
            ("stale_timeout_s", 2.0),
            ("keep_paths", 8),
            ("map_frame", "map"),
        ):
            self.declare_parameter(name, default)

        self.thresholds = RiskThresholds(
            horizon_s=float(self.get_parameter("horizon_s").value),
            dt_s=float(self.get_parameter("dt_s").value),
            samples=int(self.get_parameter("samples").value),
            max_wind_mps=float(self.get_parameter("max_wind_mps").value),
            max_tilt_deg=float(self.get_parameter("max_tilt_deg").value),
            max_position_error_m=float(self.get_parameter("max_position_error_m").value),
            max_control_saturation=float(self.get_parameter("max_control_saturation").value),
            turbulence_sigma_mps=float(self.get_parameter("turbulence_sigma_mps").value),
            turbulence_tau_s=float(self.get_parameter("turbulence_tau_s").value),
        )
        self.seed = int(self.get_parameter("seed").value)
        self.keep_paths = max(0, int(self.get_parameter("keep_paths").value))
        self.stale_timeout_s = max(0.1, float(self.get_parameter("stale_timeout_s").value))
        self.map_frame = str(self.get_parameter("map_frame").value)
        period = max(0.1, float(self.get_parameter("assessment_period_s").value))

        self._state: AircraftState | None = None
        self._mean_wind: Vec3 | None = None
        self._target: Vec3 | None = None
        self._state_time = 0.0
        self._weather_time = 0.0
        self._last_level = -1
        self._assessment_index = 0

        self.create_subscription(Odometry, "/uav_weather_risk/odom", self.on_odom, 10)
        self.create_subscription(String, "/uav_weather_risk/weather", self.on_weather, 10)
        self.create_subscription(PointStamped, "/uav_weather_risk/target", self.on_target, 10)
        self.assessment_pub = self.create_publisher(String, "/uav_weather_risk/assessment", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/uav_weather_risk/markers", 10)
        self.timer = self.create_timer(period, self.assess)
        self.get_logger().info(
            "weather risk node ready: horizon=%.1fs samples=%d"
            % (self.thresholds.horizon_s, self.thresholds.samples)
        )

    def on_odom(self, msg: Odometry) -> None:
        position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        velocity = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        )
        if not all(math.isfinite(v) for v in position + velocity):
            self.get_logger().warning("ignored non-finite odometry")
            return
        self._state = AircraftState(position_enu=position, velocity_enu=velocity)
        self._state_time = time.monotonic()

    def on_weather(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            fused = payload["fused"]
            wind = tuple(float(v) for v in fused["wind_enu"])
            if len(wind) != 3 or not all(math.isfinite(v) for v in wind):
                raise ValueError("wind_enu must contain three finite values")
            self._mean_wind = (wind[0], wind[1], wind[2])
            self._weather_time = time.monotonic()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignored malformed weather payload: {exc}")

    def on_target(self, msg: PointStamped) -> None:
        target = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        if all(math.isfinite(v) for v in target):
            self._target = target

    def _publish_invalid(self, reason: str) -> None:
        payload = {
            "valid": False,
            "level": 1,
            "level_name": "CAUTION",
            "score": 0.0,
            "reason": reason,
        }
        self.assessment_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        clear = MarkerArray()
        clear.markers.append(
            Marker(
                header=Header(
                    frame_id=self.map_frame,
                    stamp=self.get_clock().now().to_msg(),
                ),
                action=Marker.DELETEALL,
            )
        )
        self.marker_pub.publish(clear)
        if self._last_level != -1:
            self.get_logger().warning(reason)
        self._last_level = -1

    def assess(self) -> None:
        now = time.monotonic()
        if self._state is None or self._mean_wind is None:
            self._publish_invalid("waiting for odometry and fused weather")
            return
        if now - self._state_time > self.stale_timeout_s:
            self._publish_invalid("odometry is stale")
            return
        if now - self._weather_time > self.stale_timeout_s:
            self._publish_invalid("weather estimate is stale")
            return
        target_available = self._target is not None
        target = self._target or self._state.position_enu
        result = assess_risk(
            self._state,
            self._mean_wind,
            target,
            self.thresholds,
            params=AircraftParams(),
            seed=self.seed + self._assessment_index * 1009,
            keep_paths=self.keep_paths,
        )
        self._assessment_index += 1
        payload = {
            "valid": True,
            "target_available": target_available,
            "input_age_s": {
                "odom": now - self._state_time,
                "weather": now - self._weather_time,
            },
            **result.as_dict(),
        }
        self.assessment_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.marker_pub.publish(self._markers(result))
        if result.level != self._last_level:
            message = f"risk={result.level_name} score={result.score:.2f}: {result.reason}"
            if result.level >= 2:
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            self._last_level = result.level

    def _markers(self, result: RiskResult) -> MarkerArray:
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        colors = (
            (0.18, 0.49, 0.20, 1.0),
            (0.98, 0.66, 0.15, 1.0),
            (0.94, 0.42, 0.00, 1.0),
            (0.78, 0.16, 0.16, 1.0),
        )
        color = colors[result.level]
        for index, path in enumerate(result.paths):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = "predicted_paths"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.lifetime.sec = 3
            marker.scale.x = 0.06
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                color[0],
                color[1],
                color[2],
                0.35,
            )
            marker.points = [
                Point(x=point[0], y=point[1], z=point[2]) for point in path
            ]
            array.markers.append(marker)

        if self._state is None:
            return array
        position = self._state.position_enu
        sphere = Marker()
        sphere.header.frame_id = self.map_frame
        sphere.header.stamp = stamp
        sphere.ns = "risk_envelope"
        sphere.id = 100
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = position
        sphere.pose.orientation.w = 1.0
        sphere.lifetime.sec = 3
        diameter = 1.0 + 0.6 * result.level
        sphere.scale.x = sphere.scale.y = sphere.scale.z = diameter
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (
            color[0],
            color[1],
            color[2],
            0.35,
        )
        array.markers.append(sphere)

        arrow = Marker()
        arrow.header.frame_id = self.map_frame
        arrow.header.stamp = stamp
        arrow.ns = "fused_wind"
        arrow.id = 101
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.lifetime.sec = 3
        arrow.scale.x = 0.12
        arrow.scale.y = 0.30
        arrow.scale.z = 0.40
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (0.08, 0.40, 0.75, 0.95)
        end = vadd(position, vscale(self._mean_wind, 0.5))
        arrow.points = [Point(x=position[0], y=position[1], z=position[2]), Point(x=end[0], y=end[1], z=end[2])]
        array.markers.append(arrow)

        text = Marker()
        text.header.frame_id = self.map_frame
        text.header.stamp = stamp
        text.ns = "risk_label"
        text.id = 102
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = position[0]
        text.pose.position.y = position[1]
        text.pose.position.z = position[2] + 2.5
        text.pose.orientation.w = 1.0
        text.lifetime.sec = 3
        text.scale.z = 0.9
        text.color.r, text.color.g, text.color.b, text.color.a = color
        text.text = f"{result.level_name} {result.score:.2f}"
        array.markers.append(text)
        return array


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WeatherRiskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
