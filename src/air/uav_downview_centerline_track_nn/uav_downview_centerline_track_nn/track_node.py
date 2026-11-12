#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 封装节点：下视感知 -> 神经网络策略 -> 循迹控制器。

rclpy 在节点构造时才导入，离线单元测试与离屏截图脚本不需要安装 ROS。
"""

from __future__ import annotations

import numpy as np

from .controller import TrackController, TrackState
from .perception import DownviewCamera
from .policy import TrackPolicyNetwork


def main() -> None:  # pragma: no cover - 需 ROS 环境
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray

    class TrackNode(Node):
        def __init__(self) -> None:
            super().__init__("uav_downview_centerline_track_nn")
            self.declare_parameter("sim_rate", 50.0)
            self.camera = DownviewCamera()
            self.policy = TrackPolicyNetwork()
            self.controller = TrackController()
            self.state = TrackState(x=0.0, y=1.2, yaw=0.0)
            self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.pub_percept = self.create_publisher(
                Float32MultiArray, "/perception", 10)
            rate = self.get_parameter("sim_rate").value
            self.timer = self.create_timer(1.0 / rate, self._step)

        def _step(self) -> None:
            dt = 1.0 / self.get_parameter("sim_rate").value
            percept = self.camera.observe(self.state)
            speed = float(np.hypot(self.state.vx, self.state.vy))
            feat = DownviewCamera.normalize(percept["offset"],
                                           percept["heading"], speed)
            policy_out = self.policy.forward(feat)
            self.state = self.controller.step(self.state, policy_out, dt)
            self._publish()

        def _publish(self) -> None:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = float(self.state.x)
            odom.pose.pose.position.y = float(self.state.y)
            odom.twist.twist.linear.x = float(self.state.vx)
            odom.twist.twist.linear.y = float(self.state.vy)
            self.pub_odom.publish(odom)
            msg = Float32MultiArray()
            percept = self.camera.observe(self.state)
            msg.data = [float(percept["offset"]), float(percept["heading"]),
                        float(np.hypot(self.state.vx, self.state.vy))]
            self.pub_percept.publish(msg)

    rclpy.init()
    node = TrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
