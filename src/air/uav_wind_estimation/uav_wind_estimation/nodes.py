"""ROS 2 nodes: wind-triangle measurements and the least-squares estimator."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .core import estimate_wind, observability
from .scene import WindConfig, simulate_maneuver


class WindSimulator(Node):
    """Publish ground velocity, airspeed and heading of a circular survey."""

    def __init__(self) -> None:
        super().__init__("wind_sim")
        self.declare_parameter("maneuver", "circle")
        self.declare_parameter("seed", 0)
        maneuver = str(self.get_parameter("maneuver").value)
        self.data = simulate_maneuver(WindConfig(seed=int(self.get_parameter("seed").value)), maneuver)
        self.ground_pub = self.create_publisher(TwistStamped, "/uav/ground_velocity", 20)
        self.airspeed_pub = self.create_publisher(Float64, "/uav/airspeed", 20)
        self.index = 0
        self.create_timer(0.1, self.step)
        self.get_logger().info(
            f"wind_sim ready: {maneuver}, {len(self.data['headings'])} samples, "
            f"true wind {np.round(self.data['wind'], 2).tolist()} m/s"
        )

    def step(self) -> None:
        index = self.index % len(self.data["headings"])
        message = TwistStamped()
        message.header.frame_id = "map"
        message.twist.linear.x = float(self.data["ground_velocity"][index, 0])
        message.twist.linear.y = float(self.data["ground_velocity"][index, 1])
        heading = self.data["headings"][index]
        message.twist.angular.z = float(np.arctan2(heading[1], heading[0]))
        self.ground_pub.publish(message)
        airspeed = Float64()
        airspeed.data = float(self.data["airspeed"][index])
        self.airspeed_pub.publish(airspeed)
        self.index += 1


class WindEstimator(Node):
    """Collect samples and solve the wind triangle once a batch is full."""

    def __init__(self) -> None:
        super().__init__("wind_estimator")
        self.declare_parameter("batch", 60)
        self.declare_parameter("sigma_m_s", 0.2)
        self.batch = max(6, int(self.get_parameter("batch").value))
        self.sigma = float(self.get_parameter("sigma_m_s").value)
        self.ground: list[list[float]] = []
        self.headings: list[list[float]] = []
        self.create_subscription(TwistStamped, "/uav/ground_velocity", self.on_ground, 50)
        self.status_pub = self.create_publisher(String, "/uav/wind", 10)
        self.done = False
        self.get_logger().info(f"wind_estimator ready: batch {self.batch}")

    def on_ground(self, message: TwistStamped) -> None:
        if self.done:
            return
        self.ground.append([message.twist.linear.x, message.twist.linear.y])
        angle = message.twist.angular.z
        self.headings.append([np.cos(angle), np.sin(angle)])
        if len(self.ground) < self.batch:
            return
        try:
            estimate = estimate_wind(
                np.asarray(self.ground), np.asarray(self.headings), sigma_m_s=self.sigma
            )
        except ValueError as error:
            self.done = True
            self.get_logger().warning(
                f"wind not observable from these headings ({error}); stopping after {len(self.ground)} samples"
            )
            return
        payload = estimate.to_dict()
        payload["observability"] = observability(np.asarray(self.headings))
        payload["implied_airspeed_m_s"] = float(
            np.mean(np.linalg.norm(np.asarray(self.ground) - estimate.wind, axis=1))
        )
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"wind {payload['speed_m_s']:.2f} m/s at {payload['direction_deg']:.0f} deg"
        )
        self.done = True


def _spin(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_wind_sim(args=None) -> None:
    rclpy.init(args=args)
    _spin(WindSimulator())


def main_wind_estimator(args=None) -> None:
    rclpy.init(args=args)
    _spin(WindEstimator())
