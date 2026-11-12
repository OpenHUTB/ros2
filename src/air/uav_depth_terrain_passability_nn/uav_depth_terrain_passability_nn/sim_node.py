#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 封装节点：深度地形感知 -> 通行度网络 -> 决策控制器。

rclpy 在节点构造时才导入，离线单元测试与离屏截图脚本不需要安装 ROS。
"""

from __future__ import annotations

import numpy as np

from .controller import TerrainController, TerrainState
from .depth_sensor import DepthTerrainSensor
from .policy import PassabilityNetwork


def main() -> None:  # pragma: no cover - 需 ROS 环境
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray

    class TerrainNode(Node):
        def __init__(self) -> None:
            super().__init__("uav_depth_terrain_passability_nn")
            self.declare_parameter("sim_rate", 20.0)
            self.sensor = DepthTerrainSensor()
            self.policy = PassabilityNetwork()
            self.controller = TerrainController()
            self.state = TerrainState(x=0.0, y=0.0, yaw=0.0)
            self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.pub_pass = self.create_publisher(
                Float32MultiArray, "/passability", 10)
            rate = self.get_parameter("sim_rate").value
            self.timer = self.create_timer(1.0 / rate, self._step)

        def _step(self) -> None:
            dt = 1.0 / self.get_parameter("sim_rate").value
            percept = self.sensor.observe(self.state)
            feat = DepthTerrainSensor.normalize(percept["features"])
            decision = self.policy.forward(feat)
            self.state = self.controller.step(self.state, decision, dt)
            self._publish(decision)

        def _publish(self, decision: dict) -> None:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = float(self.state.x)
            odom.pose.pose.position.y = float(self.state.y)
            odom.twist.twist.linear.x = float(self.state.vx)
            odom.twist.twist.linear.y = float(self.state.vy)
            self.pub_odom.publish(odom)
            msg = Float32MultiArray()
            msg.data = list(map(float, decision["passability"])) + [
                float(decision["speed"]), float(decision["sector"])]
            self.pub_pass.publish(msg)

    rclpy.init()
    node = TerrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
