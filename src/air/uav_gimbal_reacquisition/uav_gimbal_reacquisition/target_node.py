"""ROS 2 target trajectory and occlusion publisher for the gimbal demo."""

from __future__ import annotations

import json

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .core import TargetScenario


class TargetSimNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_gimbal_target_sim")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("duration_s", 0.0)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.duration_s = max(0.0, float(self.get_parameter("duration_s").value))
        self.scenario = TargetScenario()
        self.time_s = 0.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.target_pub = self.create_publisher(PointStamped, "/uav_gimbal/target", 10)
        self.visible_pub = self.create_publisher(Bool, "/uav_gimbal/target_visible", 10)
        self.scenario_pub = self.create_publisher(String, "/uav_gimbal/scenario", 10)
        self.get_logger().info("gimbal target simulator started")

    def tick(self) -> None:
        if self.duration_s > 0.0 and self.time_s >= self.duration_s:
            self.timer.cancel()
            self.get_logger().info("target simulator completed")
            return
        sample = self.scenario.sample(self.time_s)
        stamp = self.get_clock().now().to_msg()
        target = PointStamped()
        target.header.stamp = stamp
        target.header.frame_id = "map"
        target.point.x, target.point.y, target.point.z = sample.position
        self.target_pub.publish(target)
        self.visible_pub.publish(Bool(data=sample.visible))
        self.scenario_pub.publish(
            String(
                data=json.dumps(
                    {
                        "time_s": self.time_s,
                        "visible": sample.visible,
                        "note": "occluded" if not sample.visible else "visible",
                    }
                )
            )
        )
        self.time_s += 1.0 / self.rate_hz


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
