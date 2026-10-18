"""Optional ROS 2 adapter for VRU metrics."""

from __future__ import annotations

import json
from pathlib import Path

from .metrics import evaluate_vru_tracks, load_vru_tracks


def main(args=None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError:
        print("rclpy is not installed. Use main.py for offline metrics.")
        return 2

    class VruTwinNode(Node):
        def __init__(self) -> None:
            super().__init__("vru_twin_metrics")
            self.declare_parameter("ground_truth_csv", "")
            self.declare_parameter("prediction_csv", "")
            self.declare_parameter("max_distance", 1.5)
            self.publisher = self.create_publisher(String, "~/metrics", 10)
            self.create_timer(1.0, self._run_once)
            self._done = False

        def _run_once(self) -> None:
            if self._done:
                return
            self._done = True
            gt_path = Path(str(self.get_parameter("ground_truth_csv").value))
            pred_path = Path(str(self.get_parameter("prediction_csv").value))
            if not gt_path.is_file() or not pred_path.is_file():
                self.get_logger().error("ground_truth_csv and prediction_csv must point to files")
                return
            metrics = evaluate_vru_tracks(
                load_vru_tracks(gt_path),
                load_vru_tracks(pred_path),
                max_distance=float(self.get_parameter("max_distance").value),
            )
            message = String()
            message.data = json.dumps(metrics.as_dict(), ensure_ascii=False)
            self.publisher.publish(message)
            self.get_logger().info(message.data)

    rclpy.init(args=args)
    node = VruTwinNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
