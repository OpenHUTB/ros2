#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pub_test_poses.py - 向 /drone/pose 发布 4 个测试位置 (围栏节点联调用)。"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped

TEST_POSES = [
    (50.0, 0.0, 50.0),    # 禁飞区内部 -> INSIDE
    (115.0, 0.0, 50.0),   # 距边界 15m -> WARNING
    (200.0, 0.0, 50.0),   # 安全 -> OK
    (350.0, 0.0, 50.0),   # 靠近矩形禁飞区 -> WARNING
]


def main():
    rclpy.init()
    node = rclpy.create_node('pose_test_publisher')
    pub = node.create_publisher(PoseStamped, 'drone/pose', 10)
    time.sleep(2.0)  # 等待 DDS 发现

    for x, y, z in TEST_POSES:
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.w = 1.0
        for _ in range(5):  # 重复发布确保送达
            pub.publish(msg)
            time.sleep(0.2)
        print(f'已发布测试位置: ({x}, {y}, {z})', flush=True)
        time.sleep(2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
