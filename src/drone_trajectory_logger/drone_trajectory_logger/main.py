#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹发布节点：按可配置的指令序列发送无人机速度指令（Twist）.

与旧版本的差别只在“可配置”：发布频率、指令序列、线速度/角速度系数都可以通过
ROS 参数指定，默认值与原实现完全一致（前进 -> 左转 -> 悬停，1 Hz）。
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .flight_state import DEFAULT_SEQUENCE, command_to_twist_values, parse_command_sequence


def publish_period(publish_rate):
    """把发布频率（Hz）换算成周期（秒）；非法值直接报错，不做静默兜底。"""
    rate = float(publish_rate)
    if not math.isfinite(rate) or rate <= 0.0:
        raise ValueError(
            'publish_rate must be a positive finite number (Hz), got %r' % (publish_rate,))
    return 1.0 / rate


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.cmd_vel_topic = str(self.declare_parameter('cmd_vel_topic', '/drone/cmd_vel').value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 1.0).value)
        self.linear_speed = float(self.declare_parameter('linear_speed', 1.0).value)
        self.angular_speed = float(self.declare_parameter('angular_speed', 1.0).value)
        sequence = str(self.declare_parameter('sequence', ','.join(DEFAULT_SEQUENCE)).value)
        self.commands = parse_command_sequence(sequence)

        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        period = publish_period(self.publish_rate)
        self.timer = self.create_timer(period, self.timer_callback)
        self.index = 0
        self.get_logger().info(
            '轨迹发布节点已启动。指令序列：%s' % ' -> '.join(self.commands))

    def timer_callback(self):
        command = self.commands[self.index % len(self.commands)]
        linear_x, angular_z = command_to_twist_values(
            command, self.linear_speed, self.angular_speed)

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)

        self.get_logger().info('发布: %s 指令（linear.x=%.2f, angular.z=%.2f）'
                               % (command, linear_x, angular_z))
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
