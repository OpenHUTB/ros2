#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模块总入口：一条命令跑起整个键盘遥控模块（桥接节点 + 键盘节点）。

    ros2 run uav_airsim_keyboard main
    ros2 launch uav_airsim_keyboard main.launch.py

两种方式都会创建 AirSim 桥接节点；键盘节点只在 stdin 是终端时创建
（`ros2 launch` 不转发 stdin，这是 ROS 2 的已知行为）。
"""

import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rclpy.init(args=argv)

    bridge = None
    keyboard = None
    nodes = []
    try:
        from uav_airsim_keyboard.bridge import AirSimBridge

        bridge = AirSimBridge()
        nodes.append(bridge)

        if bridge.enable_keyboard:
            if sys.stdin.isatty():
                from uav_airsim_keyboard.keyboard import KeyboardPublisher

                keyboard = KeyboardPublisher()
                nodes.append(keyboard)
            else:
                bridge.get_logger().warning(
                    'stdin 不是终端，已跳过键盘节点（ros2 launch 不转发键盘输入）。'
                    '需要键盘遥控请另开终端执行：ros2 run uav_airsim_keyboard keyboard')

        executor = MultiThreadedExecutor(num_threads=3)
        for node in nodes:
            executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            if bridge is not None:
                bridge.shutdown()
            for node in nodes:
                node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
