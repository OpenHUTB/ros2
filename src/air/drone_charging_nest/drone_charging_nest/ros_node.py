#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点入口：在 rclpy 节点生命周期内运行 MuJoCo 自主充电演示。"""
import rclpy
from rclpy.node import Node

from drone_charging_nest.app import main as run_demo


def main():
    rclpy.init()
    node = Node("drone_charging_nest")
    try:
        node.get_logger().info("starting MuJoCo drone charging/nest demo")
        run_demo()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
