"""发布仿真测距、位姿与参考速度。 / Publish simulated ranges, pose and reference speed."""
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .simulation import ANGLES, DT, MAX_RANGE, WarehouseSimulation, expert_target


class SensorNode(Node):
    def __init__(self):
        super().__init__("warehouse_ranging")
        for key, value in [("scenario", "static"), ("duration", 8.0), ("output", "/tmp/warehouse_braking")]:
            self.declare_parameter(key, value)
        self.sim = WarehouseSimulation(self.get_parameter("scenario").value, speed=1.2)
        self.scan_pub = self.create_publisher(LaserScan, "/warehouse/scan", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/warehouse/pose", 10)
        self.velocity_pub = self.create_publisher(TwistStamped, "/warehouse/reference_velocity", 10)
        self.status_pub = self.create_publisher(String, "/warehouse/status", 10)
        self.count = 0
        self.finished = False
        self.timer = self.create_timer(DT, self.tick)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        ranges = self.sim.ranges()
        target, risk = expert_target(ranges, self.sim.data.qvel[0], 1.2)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, "uav_base"
        scan.angle_min, scan.angle_max = float(ANGLES[0]), float(ANGLES[-1])
        scan.angle_increment = float(ANGLES[1] - ANGLES[0])
        scan.range_min, scan.range_max, scan.scan_time = 0.0, MAX_RANGE, DT
        scan.ranges = ranges.astype(float).tolist()
        self.scan_pub.publish(scan)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "warehouse"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, self.sim.position)
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        velocity = TwistStamped()
        velocity.header = pose.header
        velocity.twist.linear.x = target
        self.velocity_pub.publish(velocity)
        self.sim.step(target)
        self.count += 1
        if self.sim.data.time + 1e-8 >= self.get_parameter("duration").value:
            summary = {"messages": self.count, "collision": bool(self.sim.collision),
                       "sim_time_s": float(self.sim.data.time), "risk": risk,
                       "final_speed_mps": float(self.sim.data.qvel[0])}
            self.status_pub.publish(String(data=json.dumps(summary)))
            output = Path(self.get_parameter("output").value)
            output.mkdir(parents=True, exist_ok=True)
            (output / "ros_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            self.get_logger().info("仿真完成 / Simulation complete: " + json.dumps(summary))
            self.timer.cancel()
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
