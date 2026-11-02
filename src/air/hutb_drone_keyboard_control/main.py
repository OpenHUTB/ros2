#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py - 模块一键启动入口

同时启动两个节点:
    keyboard_teleop : 键盘遥控节点   -> 发布 /cmd_vel
    drone_client    : 无人机桥接节点 -> /cmd_vel -> OpenHUTB 模拟器

运行前提:
    1. OpenHUTB 模拟器已用 AIR 模式启动 (python config.py --map Town10HD?GAME=AIR)
    2. 已安装 OpenHUTB 的 Python 客户端 (pip install hutb-*.whl, 提供 airsim 模块)
    3. ROS 2 Humble 环境已 source

运行方式:
    python3 main.py
"""

import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor

from keyboard_teleop import KeyboardTeleop
from drone_client import DroneClient


def main(args=None):
    rclpy.init(args=args)

    print('=' * 58)
    print('  基于 OpenHUTB 的无人机键盘控制仿真 (ROS 2 Humble)')
    print('  节点: keyboard_teleop + drone_client')
    print('=' * 58)

    drone_node = DroneClient()      # 先启动桥接节点 (会连接并自动起飞)
    teleop_node = KeyboardTeleop()  # 再启动键盘节点

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(drone_node)
    executor.add_node(teleop_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\n收到退出信号, 正在安全降落...')
    finally:
        drone_node.destroy_node()
        teleop_node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
