"""ROS 2 Humble controller node with offline-safe imports."""

from __future__ import annotations

import argparse
from pathlib import Path
import json
import numpy as np

from rooftop_pv.models import TinyMLP


def predict_command(model: TinyMLP, cross_track: float, heading: float, distance: float) -> tuple[float, float]:
    import torch
    with torch.no_grad():
        output = model(torch.tensor([[cross_track, heading, distance]], dtype=torch.float32))[0]
    return float(output[0]), float(output[1])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ROS 2 rooftop controller / 屋顶航迹控制节点")
    parser.add_argument("--replay", type=Path, help="offline JSONL replay / 离线 JSONL 回放")
    args = parser.parse_args(argv)
    if args.replay:
        model = TinyMLP()
        rows = [json.loads(line) for line in args.replay.read_text(encoding="utf-8").splitlines()]
        for row in rows:
            command = predict_command(model, row["cross_track_error"], row["heading_error"], row["distance_to_turn"])
            print(json.dumps({"forward_speed": command[0], "yaw_rate": command[1]}))
        return 0
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
    except ImportError as exc:
        raise SystemExit("ROS 2 Humble dependencies are required for live mode / 实时模式需要 ROS 2 Humble 依赖") from exc

    class Controller(Node):
        def __init__(self) -> None:
            super().__init__("rooftop_pv_controller")
            self.declare_parameter("image_topic", "/camera/image_raw")
            self.declare_parameter("pose_topic", "/uav/pose")
            self.declare_parameter("route_topic", "/uav/route")
            self.declare_parameter("command_topic", "/cmd_vel")
            command_topic = str(self.get_parameter("command_topic").value or "/cmd_vel")
            self.publisher = self.create_publisher(Twist, command_topic, 10)
            self.timer = self.create_timer(0.1, self.publish_zero)

        def publish_zero(self) -> None:
            self.publisher.publish(Twist())

    rclpy.init(args=argv)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
