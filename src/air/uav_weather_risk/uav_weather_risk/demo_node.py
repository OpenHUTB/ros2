"""ROS 2 demo publisher: synthetic weather, UAV response, and target."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .core.aircraft import AircraftModel, AircraftParams, AircraftState
from .core.scenario import DEFAULT_ROUTE, GustFrontScenario
from .core.types import vnorm, vsub
from .core.weather import fuse_sources


class WeatherDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_weather_demo")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("duration_s", 0.0)
        self.declare_parameter("seed", 42)
        self.declare_parameter("waypoint_tolerance_m", 2.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.seed = int(self.get_parameter("seed").value)
        self.tolerance = max(0.1, float(self.get_parameter("waypoint_tolerance_m").value))
        self.dt_s = 1.0 / self.rate_hz

        self.odom_pub = self.create_publisher(Odometry, "/uav_weather_risk/odom", 10)
        self.weather_pub = self.create_publisher(String, "/uav_weather_risk/weather", 10)
        self.target_pub = self.create_publisher(PointStamped, "/uav_weather_risk/target", 10)
        self.timer = self.create_timer(self.dt_s, self.tick)

        self.scenario = GustFrontScenario(self.seed)
        self.vehicle = AircraftModel(
            AircraftParams(),
            AircraftState(position_enu=(0.0, 0.0, 30.0)),
        )
        self.route = DEFAULT_ROUTE
        self.target_index = 0
        self.target = self.route[0]
        self.sim_time_s = 0.0
        self._finished = False
        self.get_logger().info(
            "weather demo started: route=%d waypoints, duration=%s"
            % (len(self.route), "unbounded" if self.duration_s == 0.0 else f"{self.duration_s:.1f}s")
        )

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def tick(self) -> None:
        if self._finished:
            return
        if self.duration_s > 0.0 and self.sim_time_s >= self.duration_s:
            self._finished = True
            self.timer.cancel()
            self.get_logger().info("weather demo completed")
            return

        true_wind = self.scenario.step(self.dt_s)
        self.vehicle.step(self.dt_s, true_wind, self.target)
        self.sim_time_s += self.dt_s
        state = self.vehicle.state
        if vnorm(vsub(self.target, state.position_enu)) <= self.tolerance:
            self.target_index = (self.target_index + 1) % len(self.route)
            self.target = self.route[self.target_index]

        stamp = self._stamp()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = state.position_enu[0]
        odom.pose.pose.position.y = state.position_enu[1]
        odom.pose.pose.position.z = state.position_enu[2]
        roll = math.radians(state.roll_deg)
        pitch = math.radians(state.pitch_deg)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        odom.pose.pose.orientation.w = cr * cp
        odom.pose.pose.orientation.x = sr * cp
        odom.pose.pose.orientation.y = cr * sp
        odom.pose.pose.orientation.z = -sr * sp
        odom.twist.twist.linear.x = state.velocity_enu[0]
        odom.twist.twist.linear.y = state.velocity_enu[1]
        odom.twist.twist.linear.z = state.velocity_enu[2]
        self.odom_pub.publish(odom)

        target = PointStamped()
        target.header.stamp = stamp
        target.header.frame_id = "map"
        target.point.x, target.point.y, target.point.z = self.target
        self.target_pub.publish(target)

        fused = fuse_sources(self.scenario.sources())
        payload = {
            "sim_time_s": self.sim_time_s,
            "truth_wind_enu": list(true_wind),
            "fused": fused.as_dict(),
        }
        self.weather_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WeatherDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
