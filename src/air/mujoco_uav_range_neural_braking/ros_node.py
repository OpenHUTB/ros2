"""ROS 2 无人机测距节点。 ROS 2 UAV ranging and braking node."""
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray

from ros_messages import (
    TOPIC_BRAKING_ACCEL,
    TOPIC_FRONT_RANGE,
    TOPIC_STATE,
    build_message_payload,
)
from simulation import Scenario, Simulation


class UavRangeNode(Node):
    """以固定周期推进仿真并发布传感器与控制状态。 Advance the simulation and publish state."""

    def __init__(self, controller="mlp", model_path=None, period=0.01):
        super().__init__("uav_range_braking")
        self.scenario = Scenario()
        self.simulation = Simulation(self.scenario, controller=controller, model_path=model_path)
        self.finished = False
        self.range_pub = self.create_publisher(Float32, TOPIC_FRONT_RANGE, 10)
        self.control_pub = self.create_publisher(Float32, TOPIC_BRAKING_ACCEL, 10)
        self.state_pub = self.create_publisher(Float64MultiArray, TOPIC_STATE, 10)
        self.timer = self.create_timer(period, self.tick)

    def tick(self):
        row = self.simulation.step()
        payload = build_message_payload(row)
        range_msg = Float32()
        range_msg.data = payload.front_range_m
        control_msg = Float32()
        control_msg.data = payload.braking_accel_m_s2
        state_msg = Float64MultiArray()
        state_msg.data = list(payload.state)
        self.range_pub.publish(range_msg)
        self.control_pub.publish(control_msg)
        self.state_pub.publish(state_msg)
        if self.simulation.data.time >= self.scenario.duration:
            self.timer.cancel()
            self.finished = True
            self.get_logger().info("仿真时长已结束 / Simulation duration reached")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("reference", "range", "mlp"), default="mlp")
    parser.add_argument("--model", type=Path, default=None)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = UavRangeNode(args.controller, args.model)
    try:
        # 回调返回后再关闭 ROS，避免 shutdown 等待当前回调完成而相互阻塞。
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
