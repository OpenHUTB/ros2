#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点入口：在 rclpy 节点生命周期内运行单臂抓取演示。

ros2 launch 的 Node action 会追加 `--ros-args -r __node:=...`，这里先用
remove_ros_args 去掉 ROS 参数，再把其余参数（--record/--fps/--policy）
透传给 app.main()（其内部用 parse_known_args，兼容未知参数）。
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from pi0_mujoco_grasp.app import main as run_demo


def main():
    rclpy.init()
    node = Node("pi0_mujoco_grasp")
    try:
        extra = remove_ros_args(sys.argv)[1:]
        node.get_logger().info("starting MuJoCo pick-place / pi0 demo: %s", " ".join(extra))
        try:
            run_demo(extra)
        except SystemExit as exc:  # --policy pi0 等主动退出场景
            node.get_logger().error("MuJoCo demo exited: %s", exc)
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
