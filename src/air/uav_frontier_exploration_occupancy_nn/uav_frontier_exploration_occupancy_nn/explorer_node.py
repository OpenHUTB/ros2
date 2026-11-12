#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 封装节点：占据栅格建图 -> 前沿提取 -> 网络选点 -> 导航。

rclpy 在节点构造时才导入，离线单元测试与离屏截图脚本不需要安装 ROS。
"""

from __future__ import annotations

from .explorer import FrontierExplorer


def main() -> None:  # pragma: no cover - 需 ROS 环境
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32MultiArray

    class ExplorerNode(Node):
        def __init__(self) -> None:
            super().__init__("uav_frontier_exploration_occupancy_nn")
            self.declare_parameter("sim_rate", 20.0)
            self.explorer = FrontierExplorer()
            self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.pub_frontier = self.create_publisher(
                Float32MultiArray, "/frontier", 10)
            rate = self.get_parameter("sim_rate").value
            self.timer = self.create_timer(1.0 / rate, self._step)

        def _step(self) -> None:
            status = self.explorer.step()
            self._publish(status)

        def _publish(self, status: dict) -> None:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = float(status["x"])
            odom.pose.pose.position.y = float(status["y"])
            self.pub_odom.publish(odom)

            target = status["target"]
            tx = float(target["x"]) if target else 0.0
            ty = float(target["y"]) if target else 0.0
            score = float(status["scores"].max()) if len(status["scores"]) else 0.0
            msg = Float32MultiArray()
            msg.data = [tx, ty, score, float(status["n_candidates"]),
                        float(status["free"]), float(status["unknown"])]
            self.pub_frontier.publish(msg)

    rclpy.init()
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
