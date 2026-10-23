"""ROS 2 无人机测距节点。 ROS 2 UAV ranging and braking node."""
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray

from simulation import Scenario, Simulation


class UavRangeNode(Node):
    """以固定周期推进仿真并发布传感器与控制状态。 Advance the simulation and publish state."""

    def __init__(self, controller="mlp", model_path=None, period=0.01):
        super().__init__("uav_range_braking")
        self.simulation = Simulation(Scenario(), controller=controller, model_path=model_path)
        self.range_pub = self.create_publisher(Float32, "uav/front_range_m", 10)
        self.control_pub = self.create_publisher(Float32, "uav/braking_accel_m_s2", 10)
        self.state_pub = self.create_publisher(Float64MultiArray, "uav/state", 10)
        self.timer = self.create_timer(period, self.tick)

    def tick(self):
        row = self.simulation.step()
        range_msg = Float32()
        range_msg.data = float(row["sensor_range_m"])
        control_msg = Float32()
        control_msg.data = float(row["accel_m_s2"])
        state_msg = Float64MultiArray()
        state_msg.data = [row["time_s"], row["sensor_range_m"], row["closing_speed_m_s"], row["height_m"]]
        self.range_pub.publish(range_msg)
        self.control_pub.publish(control_msg)
        self.state_pub.publish(state_msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("range", "mlp"), default="mlp")
    parser.add_argument("--model", type=Path, default=None)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = UavRangeNode(args.controller, args.model)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
