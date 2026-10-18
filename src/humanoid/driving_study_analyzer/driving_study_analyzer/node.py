"""Optional ROS 2 adapter: analyze one session and publish its JSON report."""

from __future__ import annotations

import json
from pathlib import Path

from .analyzer import analyze_files


def main(args=None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
    except ImportError:
        print("rclpy is not installed. Use main.py for offline analysis.")
        return 2

    class DrivingStudyNode(Node):
        def __init__(self) -> None:
            super().__init__("driving_study_analyzer")
            self.declare_parameter("driving_csv", "")
            self.declare_parameter("gaze_csv", "")
            self.declare_parameter("output_json", "driving_study_report.json")
            qos = QoSProfile(depth=10)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.publisher = self.create_publisher(
                String, "/driving_study_analyzer/report", qos
            )
            self.create_timer(1.0, self._run_once)
            self._done = False

        def _run_once(self) -> None:
            if self._done:
                return
            driving = Path(str(self.get_parameter("driving_csv").value))
            gaze = Path(str(self.get_parameter("gaze_csv").value))
            if not driving.is_file() or not gaze.is_file():
                self.get_logger().error("driving_csv and gaze_csv must point to files")
                return
            self._done = True
            report = analyze_files(driving, gaze)
            output = Path(str(self.get_parameter("output_json").value))
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            message = String()
            message.data = json.dumps(report, ensure_ascii=False)
            self.publisher.publish(message)
            self.get_logger().info("report written to %s" % output)

    rclpy.init(args=args)
    node = DrivingStudyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
