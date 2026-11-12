#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 Humble 封装节点：订阅 /cmd_vel，驱动离线四旋翼动力学并发布里程计。

说明:
    本节点仅在 ROS 2 环境下运行；rclpy 在节点构造时才导入，
    因此离线单元测试与离屏截图脚本不需要安装 ROS。
"""

from __future__ import annotations

import numpy as np

from .dynamics import Quadrotor, State
from .controller import TeleopController


def cmd_vel_to_cmd(twist) -> dict:
    """geometry_msgs/Twist -> 离线动力学遥控指令字典。"""
    return {
        "throttle": float(np.clip(twist.linear.z, -1.0, 1.0)),
        "pitch": float(np.clip(twist.linear.x, -1.0, 1.0)),
        "roll": float(np.clip(twist.linear.y, -1.0, 1.0)),
        "yaw": float(np.clip(twist.angular.z, -1.0, 1.0)),
    }


def main() -> None:  # pragma: no cover - 需 ROS 环境
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu

    class TeleopNode(Node):
        def __init__(self) -> None:
            super().__init__("uav_rigidbody_keyboard_teleop")
            self.declare_parameter("sim_rate", 250.0)
            self.robot = Quadrotor()
            self.controller = TeleopController()
            self.state = State()
            self._latest_cmd = {"throttle": 0.0, "pitch": 0.0,
                                "roll": 0.0, "yaw": 0.0}
            self.sub_cmd = self.create_subscription(
                Twist, "/cmd_vel", self._cmd_cb, 10)
            self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.pub_imu = self.create_publisher(Imu, "/imu", 10)
            rate = self.get_parameter("sim_rate").value
            self.timer = self.create_timer(1.0 / rate, self._step)

        def _cmd_cb(self, twist: Twist) -> None:
            self._latest_cmd = cmd_vel_to_cmd(twist)

        def _step(self) -> None:
            dt = 1.0 / self.get_parameter("sim_rate").value
            wrench = self.controller.update(self.state, self._latest_cmd)
            self.state = self.robot.step(self.state, wrench, dt)
            self._publish_odom()
            self._publish_imu()

        def _publish_odom(self) -> None:
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose.pose.position.x = float(self.state.pos[0])
            msg.pose.pose.position.y = float(self.state.pos[1])
            msg.pose.pose.position.z = float(self.state.pos[2])
            msg.twist.twist.linear.x = float(self.state.vel[0])
            msg.twist.twist.linear.z = float(self.state.vel[2])
            self.pub_odom.publish(msg)

        def _publish_imu(self) -> None:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.angular_velocity.x = float(self.state.angular_vel[0])
            msg.angular_velocity.y = float(self.state.angular_vel[1])
            msg.angular_velocity.z = float(self.state.angular_vel[2])
            self.pub_imu.publish(msg)

    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
