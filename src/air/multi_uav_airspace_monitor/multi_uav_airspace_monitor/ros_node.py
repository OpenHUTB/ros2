"""Optional ROS 2 wrapper.

The core monitor is deliberately ROS-free. This node accepts a JSON state
message so it can be fed by AirSim, CarlaAir, PX4 or a recorded replay.
"""

from __future__ import annotations

import json
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .monitor import UAVTrack, Zone, evaluate_scenario


class AirspaceMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("multi_uav_airspace_monitor")
        self.declare_parameter("state_topic", "/airspace/uav_states")
        self.declare_parameter("alert_topic", "/airspace/alerts")
        self.declare_parameter("zones_json", "[]")
        self.declare_parameter("history_s", 10.0)
        self.declare_parameter("warning_distance_m", 30.0)
        self.declare_parameter("critical_distance_m", 10.0)

        self.history: dict[str, deque] = defaultdict(lambda: deque(maxlen=400))
        self.zones = [
            Zone(
                name=item["name"],
                polygon=item["polygon"],
                floor_z=item.get("floor_z", 0.0),
                ceiling_z=item.get("ceiling_z", 120.0),
                buffer_m=item.get("buffer_m", 0.0),
            )
            for item in json.loads(self.get_parameter("zones_json").value)
        ]
        self.publisher = self.create_publisher(
            String, self.get_parameter("alert_topic").value, 10
        )
        self.subscription = self.create_subscription(
            String,
            self.get_parameter("state_topic").value,
            self.on_state,
            10,
        )

    def on_state(self, message: String) -> None:
        payload = json.loads(message.data)
        stamp = float(payload.get("stamp", 0.0))
        for item in payload.get("uavs", []):
            self.history[item["id"]].append(
                (stamp, float(item["x"]), float(item["y"]), float(item["z"]))
            )
        self.evaluate_and_publish()

    def evaluate_and_publish(self) -> None:
        history_s = float(self.get_parameter("history_s").value)
        tracks = []
        now = max((values[-1][0] for values in self.history.values() if values), default=0.0)
        for uav_id, values in self.history.items():
            recent = [value for value in values if value[0] >= now - history_s]
            if len(recent) < 2:
                continue
            tracks.append(
                UAVTrack(
                    uav_id=uav_id,
                    times=[value[0] for value in recent],
                    positions=[[value[1], value[2], value[3]] for value in recent],
                )
            )
        if not tracks:
            return

        scenario = {
            "horizon_s": history_s,
            "warning_distance_m": self.get_parameter("warning_distance_m").value,
            "critical_distance_m": self.get_parameter("critical_distance_m").value,
            "zones": [
                {
                    "name": zone.name,
                    "polygon": zone.polygon.tolist(),
                    "floor_z": zone.floor_z,
                    "ceiling_z": zone.ceiling_z,
                    "buffer_m": zone.buffer_m,
                }
                for zone in self.zones
            ],
            "uavs": [
                {
                    "id": track.uav_id,
                    "times": track.times.tolist(),
                    "positions": track.positions.tolist(),
                }
                for track in tracks
            ],
        }
        result = evaluate_scenario(scenario)
        result["stamp"] = round(now, 6)
        self.publisher.publish(String(data=json.dumps(result, ensure_ascii=False)))
        if result["status"] == "critical":
            self.get_logger().warning(
                f"airspace critical: {len(result['conflicts'])} conflict(s), "
                f"{len(result['alerts'])} alert(s)"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirspaceMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
