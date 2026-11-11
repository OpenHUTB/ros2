"""ROS 2 gimbal tracker, loss/reacquisition state machine, and RViz markers."""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .core import (
    GimbalConfig,
    GimbalController,
    Metrics,
    TargetSample,
    boresight_direction,
)


class GimbalTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_gimbal_tracker")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("uav_x", 0.0)
        self.declare_parameter("uav_y", 0.0)
        self.declare_parameter("uav_z", 20.0)
        self.declare_parameter("loss_frames", 3)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        config = GimbalConfig(
            uav_position=(
                float(self.get_parameter("uav_x").value),
                float(self.get_parameter("uav_y").value),
                float(self.get_parameter("uav_z").value),
            ),
            loss_frames=int(self.get_parameter("loss_frames").value),
        )
        self.config = config
        self.controller = GimbalController(config)
        self.metrics = Metrics()
        self.target: tuple[float, float, float] | None = None
        self.visible = False
        self.time_s = 0.0
        self.last_state = None
        self.last_log_s = -1.0
        self.create_subscription(PointStamped, "/uav_gimbal/target", self.on_target, 10)
        self.create_subscription(Bool, "/uav_gimbal/target_visible", self.on_visible, 10)
        self.observation_pub = self.create_publisher(String, "/uav_gimbal/observation", 10)
        self.status_pub = self.create_publisher(String, "/uav_gimbal/status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/uav_gimbal/markers", 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info("gimbal target-loss tracker started")

    def on_target(self, msg: PointStamped) -> None:
        target = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        if all(math.isfinite(value) for value in target):
            self.target = target

    def on_visible(self, msg: Bool) -> None:
        self.visible = bool(msg.data)

    def tick(self) -> None:
        if self.target is None:
            return
        dt = 1.0 / self.rate_hz
        self.time_s += dt
        observation = self.controller.step(
            TargetSample(self.time_s, self.target, self.visible), dt
        )
        self.metrics.update(observation)
        self.observation_pub.publish(
            String(data=json.dumps(observation.__dict__, ensure_ascii=False))
        )
        if self.last_state != observation.state:
            self.get_logger().info(
                f"state={observation.state} detected={observation.detected} "
                f"los={observation.los_error_deg:.2f} deg"
            )
            self.last_state = observation.state
        if self.time_s - self.last_log_s >= 1.0:
            summary = self.metrics.summary()
            self.status_pub.publish(String(data=json.dumps(summary, ensure_ascii=False)))
            self.last_log_s = self.time_s
        self.marker_pub.publish(self._markers(observation))

    def _marker(self, marker_type: int, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uav_gimbal_reacquisition"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    def _markers(self, observation) -> MarkerArray:
        array = MarkerArray()
        uav = self.config.uav_position
        target = self.target
        if target is None:
            return array

        uav_marker = self._marker(Marker.SPHERE, 0)
        uav_marker.pose.position.x, uav_marker.pose.position.y, uav_marker.pose.position.z = uav
        uav_marker.scale.x = uav_marker.scale.y = uav_marker.scale.z = 1.2
        uav_marker.color.r, uav_marker.color.g, uav_marker.color.b, uav_marker.color.a = (
            0.1,
            0.45,
            0.95,
            0.9,
        )
        array.markers.append(uav_marker)

        target_marker = self._marker(Marker.SPHERE, 1)
        target_marker.pose.position.x, target_marker.pose.position.y, target_marker.pose.position.z = target
        target_marker.scale.x = target_marker.scale.y = target_marker.scale.z = 1.2
        if observation.detected:
            target_marker.color.r, target_marker.color.g, target_marker.color.b = 0.2, 0.9, 0.3
        else:
            target_marker.color.r, target_marker.color.g, target_marker.color.b = 0.95, 0.2, 0.2
        target_marker.color.a = 1.0
        array.markers.append(target_marker)

        direction = boresight_direction(self.controller.yaw_deg, self.controller.pitch_deg)
        endpoint = tuple(uav[i] + direction[i] * 12.0 for i in range(3))
        arrow = self._marker(Marker.ARROW, 2)
        arrow.points = [Point(x=uav[0], y=uav[1], z=uav[2]), Point(x=endpoint[0], y=endpoint[1], z=endpoint[2])]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.12, 0.28, 0.35
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.95, 0.75, 0.1, 0.9
        array.markers.append(arrow)

        fov = self._marker(Marker.LINE_LIST, 3)
        for sign in (-1.0, 1.0):
            edge_yaw = self.controller.yaw_deg + sign * self.config.hfov_deg / 2.0
            edge = boresight_direction(edge_yaw, self.controller.pitch_deg)
            fov.points.extend(
                [
                    Point(x=uav[0], y=uav[1], z=uav[2]),
                    Point(
                        x=uav[0] + edge[0] * 10.0,
                        y=uav[1] + edge[1] * 10.0,
                        z=uav[2] + edge[2] * 10.0,
                    ),
                ]
            )
        fov.scale.x = 0.05
        fov.color.r, fov.color.g, fov.color.b, fov.color.a = 0.25, 0.7, 0.95, 0.55
        array.markers.append(fov)

        text = self._marker(Marker.TEXT_VIEW_FACING, 4)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = (
            target[0],
            target[1],
            target[2] + 2.0,
        )
        text.scale.z = 1.0
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
        text.text = f"{observation.state}  LOS {observation.los_error_deg:.1f} deg"
        array.markers.append(text)
        return array


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
