"""ROS 2 adapter for VRU conflict-risk maps."""

from __future__ import annotations

import json
from pathlib import Path

from .risk import evaluate_conflicts, load_ego_track, load_vru_tracks
from .swept import evaluate_swept_conflicts


def main(args=None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError:
        print("rclpy is not installed. Use main.py for offline evaluation.")
        return 2

    class RiskNode(Node):
        def __init__(self) -> None:
            super().__init__("vru_conflict_risk_map")
            self.declare_parameter("ego_csv", "")
            self.declare_parameter("vru_csv", "")
            self.declare_parameter("analysis_mode", "point")
            self.declare_parameter("ego_radius", 1.0)
            self.declare_parameter("caution_clearance", 1.0)
            self.declare_parameter("horizon", 5.0)
            self.publisher = self.create_publisher(String, "~/report", 10)
            self.create_timer(1.0, self._run_once)
            self._done = False

        def _run_once(self) -> None:
            if self._done:
                return
            ego = Path(str(self.get_parameter("ego_csv").value))
            vru = Path(str(self.get_parameter("vru_csv").value))
            if not ego.is_file() or not vru.is_file():
                self.get_logger().error("ego_csv and vru_csv must point to files")
                return
            self._done = True
            ego_track = load_ego_track(ego)
            vru_tracks = load_vru_tracks(vru)
            mode = str(self.get_parameter("analysis_mode").value)
            if mode == "swept":
                report = evaluate_swept_conflicts(
                    ego_track,
                    vru_tracks,
                    ego_radius_m=float(self.get_parameter("ego_radius").value),
                    caution_clearance_m=float(
                        self.get_parameter("caution_clearance").value),
                    horizon_s=float(self.get_parameter("horizon").value),
                )
            else:
                report = evaluate_conflicts(ego_track, vru_tracks)
            message = String()
            message.data = json.dumps(report, ensure_ascii=False)
            self.publisher.publish(message)
            self.get_logger().info(message.data)

    rclpy.init(args=args)
    node = RiskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
