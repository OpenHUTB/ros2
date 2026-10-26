"""ROS 学习控制接口。 / ROS learned-control interface."""

import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .model import RiskSpeedMLP
from .simulation import expert_target


class LearnedControllerNode(Node):
    """将已训练模型接入测距与位姿话题。 / Connect a trained model to ranges and pose."""

    def __init__(self):
        super().__init__("warehouse_learned_controller")
        self.declare_parameter("model", "model/risk_speed_mlp.npz")
        self.declare_parameter("duration", 8.0)
        self.model = RiskSpeedMLP.load(Path(self.get_parameter("model").value))
        self.duration = float(self.get_parameter("duration").value)
        self.started_at = self.get_clock().now()
        self.previous_ranges = None
        self.speed = 0.0
        self.last_pose = None
        self.last_time = None
        self.scan_sub = self.create_subscription(LaserScan, "/warehouse/scan", self.on_scan, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/warehouse/pose", self.on_pose, 10)
        self.velocity_pub = self.create_publisher(TwistStamped, "/warehouse/learned_velocity", 10)
        self.status_pub = self.create_publisher(String, "/warehouse/learned_status", 10)

    def on_pose(self, message):
        now = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.last_pose is not None and now > self.last_time:
            self.speed = max(0.0, (message.pose.position.x - self.last_pose) / (now - self.last_time))
        self.last_pose, self.last_time = message.pose.position.x, now

    def on_scan(self, message):
        ranges = np.asarray(message.ranges, dtype=np.float32)
        if ranges.size != 9 or not np.isfinite(ranges).all():
            self.get_logger().warning("测距输入无效 / Invalid range input")
            return
        closing = np.zeros(9) if self.previous_ranges is None else np.clip(
            (self.previous_ranges - ranges) / max(message.scan_time, 0.02), -10, 10)
        features = np.concatenate([ranges, closing, [self.speed, 1.2]])[None, :]
        probability, speed = self.model.predict(features)
        safety_speed, reference_risk = expert_target(ranges, self.speed, 1.2)
        command = min(float(speed[0]), safety_speed)
        output = TwistStamped()
        output.header = message.header
        output.twist.linear.x = command
        self.velocity_pub.publish(output)
        self.status_pub.publish(String(data=json.dumps({
            "risk": int(probability[0].argmax()), "reference_risk": reference_risk,
            "risk_probability": probability[0].round(4).tolist(),
            "model_speed_mps": float(speed[0]), "target_speed_mps": command,
        })))
        self.previous_ranges = ranges.copy()


def main(args=None):
    rclpy.init(args=args)
    node = LearnedControllerNode()
    try:
        while rclpy.ok():
            elapsed = (node.get_clock().now() - node.started_at).nanoseconds / 1e9
            if elapsed >= node.duration:
                node.get_logger().info("学习控制完成 / Learned control complete")
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
