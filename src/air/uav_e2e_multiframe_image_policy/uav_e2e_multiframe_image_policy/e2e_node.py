#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 封装节点：多帧下视图像 -> 端到端策略 -> 前飞/偏航指令。

rclpy 在节点构造时才导入，离线单元测试与离屏截图脚本不需要安装 ROS。
"""

from __future__ import annotations

import math

from .runner import E2ERunner


def main() -> None:  # pragma: no cover - 需 ROS 环境
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray

    class E2ENode(Node):
        def __init__(self) -> None:
            super().__init__("uav_e2e_multiframe_image_policy")
            self.declare_parameter("sim_rate", 20.0)
            self.runner = E2ERunner()
            self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.pub_cmd = self.create_publisher(
                Float32MultiArray, "/e2e_policy_cmd", 10)
            rate = self.get_parameter("sim_rate").value
            self.timer = self.create_timer(1.0 / rate, self._step)

        def _step(self) -> None:
            status = self.runner.step()
            self._publish(status)

        def _publish(self, status: dict) -> None:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = float(status["x"])
            odom.pose.pose.position.y = float(status["y"])
            yaw = float(status["yaw"])
            # 绕 z 轴的偏航四元数，保证 /odom 的 orientation 是有效单位四元数
            odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.pub_odom.publish(odom)

            msg = Float32MultiArray()
            msg.data = [
                float(status["forward"]),
                float(status["yaw_cmd"]),
                float(status["offset"]),
                float(status["yaw"]),
            ]
            self.pub_cmd.publish(msg)

    rclpy.init()
    node = E2ENode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
