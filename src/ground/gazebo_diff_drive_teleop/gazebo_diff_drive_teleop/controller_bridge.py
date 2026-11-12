#!/usr/bin/env python3
"""适配标准 /cmd_vel、/odom 与 ros2_control 控制器原生话题。

ROS 2 Jazzy 的 diff_drive_controller 使用：
  - 订阅 /diff_drive_controller/cmd_vel  (geometry_msgs/TwistStamped)
  - 发布 /diff_drive_controller/odom     (nav_msgs/Odometry)

本节点把示教节点发布的标准 geometry_msgs/Twist 转换成 TwistStamped 发给
控制器，并把控制器里程计转发到 /odom。这样示教节点和外部工具都可以继续
使用 ROS 2 常见的 /cmd_vel、/odom 接口。
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class ControllerBridge(Node):
    """双向转发速度指令和里程计。"""

    def __init__(self):
        super().__init__('controller_bridge')

        self.declare_parameter('input_cmd_vel_topic', 'cmd_vel')
        self.declare_parameter(
            'output_cmd_vel_topic', 'diff_drive_controller/cmd_vel')
        self.declare_parameter(
            'input_odom_topic', 'diff_drive_controller/odom')
        self.declare_parameter('output_odom_topic', 'odom')

        input_cmd = str(self.get_parameter('input_cmd_vel_topic').value)
        output_cmd = str(self.get_parameter('output_cmd_vel_topic').value)
        input_odom = str(self.get_parameter('input_odom_topic').value)
        output_odom = str(self.get_parameter('output_odom_topic').value)

        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.cmd_publisher = self.create_publisher(
            TwistStamped, output_cmd, cmd_qos)
        self.odom_publisher = self.create_publisher(Odometry, output_odom, 10)

        self.create_subscription(Twist, input_cmd, self._on_cmd_vel, cmd_qos)
        self.create_subscription(Odometry, input_odom, self._on_odom, 10)

        self.get_logger().info(
            f'速度桥: /{input_cmd} (Twist) -> /{output_cmd} (TwistStamped)')
        self.get_logger().info(
            f'里程计桥: /{input_odom} -> /{output_odom}')

    def _on_cmd_vel(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_footprint'
        stamped.twist = msg
        self.cmd_publisher.publish(stamped)

    def _on_odom(self, msg: Odometry):
        self.odom_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
