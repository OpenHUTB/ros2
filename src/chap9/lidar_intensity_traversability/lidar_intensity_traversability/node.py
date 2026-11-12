"""ROS 2 节点：订阅 /scan，按回波强度给出的可通行性发布 /cmd_vel。"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from lidar_intensity_traversability import core


class IntensityTraversabilityNode(Node):
    def __init__(self):
        super().__init__("lidar_intensity_traversability")
        self.declare_parameter("max_speed", 0.35)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("slow_distance", 1.20)
        self.declare_parameter("ref_intensity", 1200.0)
        self.declare_parameter("uncertainty", 0.60)
        self.declare_parameter("log_period", 0.5)

        self.max_speed = float(self.get_parameter("max_speed").value)
        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.ref_intensity = float(self.get_parameter("ref_intensity").value)
        self.uncertainty = float(self.get_parameter("uncertainty").value)
        self.log_period = float(self.get_parameter("log_period").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.decision_pub = self.create_publisher(String, "/traversability/decision", 10)
        self.sub = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.last_log = 0.0
        self.frames = 0
        self.actions = {}
        self.get_logger().info(
            "intensity traversability ready: fov=%.0f stop=%.2fm uncertainty=%.2f"
            % (self.fov_deg, self.stop_distance, self.uncertainty))

    def on_scan(self, msg):
        beams = core.build_beams(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.intensities,
            ref=self.ref_intensity, uncertainty=self.uncertainty,
            range_max=msg.range_max if msg.range_max > 0.0 else 12.0)
        cmd = core.choose_heading(
            beams, fov_deg=self.fov_deg, stop_distance=self.stop_distance,
            slow_distance=self.slow_distance, max_speed=self.max_speed)

        twist = Twist()
        twist.linear.x = float(cmd["linear"])
        twist.angular.z = float(cmd["angular"])
        self.pub.publish(twist)
        self.decision_pub.publish(String(data="%.6f,%.6f,%s" % (
            cmd["min_effective_front"], cmd["heading_deg"], cmd["action"])))

        self.frames += 1
        self.actions[cmd["action"]] = self.actions.get(cmd["action"], 0) + 1

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log >= self.log_period:
            self.last_log = now
            weak = sum(1 for b in beams if b["confidence"] < 0.25)
            self.get_logger().info(
                "action=%-10s heading=%+6.1f deg front_eff=%5.2fm "
                "beams=%3d weak=%2d v=%.2f w=%+.2f"
                % (cmd["action"], cmd["heading_deg"], cmd["min_effective_front"],
                   len(beams), weak, twist.linear.x, twist.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = IntensityTraversabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("frames=%d actions=%s" % (node.frames, node.actions))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
