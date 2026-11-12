#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点入口：记录日志后调用 DAVE 启动脚本（实际仿真仍用 ROS 1 Noetic 运行）。

ROS 2 参数（--ros-args / -r / remap）会被 remove_ros_args 过滤，其余
`demo|valve|plug [headless]` 参数透传给 run_demo.sh。
"""
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from dave_underwater_sim.app import _share_dir, demo_args


def main():
    rclpy.init()
    node = Node("dave_underwater_sim")
    try:
        extra = demo_args(remove_ros_args(sys.argv)[1:])
        script = _share_dir() + "/run_demo.sh"
        node.get_logger().info("launching DAVE underwater demo: %s", " ".join(extra))
        rc = subprocess.run(["bash", script] + extra, check=False).returncode
        if rc != 0:
            node.get_logger().error("DAVE underwater demo exited with code %d", rc)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
