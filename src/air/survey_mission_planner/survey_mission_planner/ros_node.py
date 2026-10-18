"""Optional ROS 2 wrapper that publishes a survey Path and summary."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from .camera import CameraSpec
from .planner import SurveyPlanner


class SurveyMissionPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("survey_mission_planner")
        self.declare_parameter("target_gsd_m", 0.012)
        self.declare_parameter("forward_overlap", 0.8)
        self.declare_parameter("side_overlap", 0.7)
        self.declare_parameter("speed_mps", 8.0)
        self.path_pub = self.create_publisher(Path, "/survey/path", 10)
        self.summary_pub = self.create_publisher(String, "/survey/summary", 10)
        self.create_subscription(String, "/survey/plan_request", self.on_request, 1)

    def on_request(self, message: String) -> None:
        payload = json.loads(message.data)
        camera = CameraSpec(**payload["camera"])
        planner = SurveyPlanner(
            camera=camera,
            target_gsd_m=float(payload["target_gsd_m"]),
            forward_overlap=float(payload.get("forward_overlap", 0.8)),
            side_overlap=float(payload.get("side_overlap", 0.7)),
            speed_mps=float(self.get_parameter("speed_mps").value),
            origin_lat=float(payload.get("origin_lat", 28.22)),
            origin_lon=float(payload.get("origin_lon", 112.94)),
        )
        plan = planner.plan(np.asarray(payload["polygon"], dtype=float))
        path = Path()
        path.header.frame_id = "world"
        for east, north in plan.waypoints_enu:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(east)
            pose.pose.position.y = float(north)
            pose.pose.position.z = float(plan.altitude_m)
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.summary_pub.publish(String(data=json.dumps(plan.as_dict(), ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SurveyMissionPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
