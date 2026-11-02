#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geofence_detector/main.py - ROS2 节点入口

订阅无人机位置 (geometry_msgs/PoseStamped), 对所有禁飞区做实时检测:
    - OK:      每 10s 打印一次心跳
    - WARNING: 打印预警 + 发布 /drone/geofence_status (String)
    - INSIDE:  打印告急 + 发布状态, 建议立即爬升脱离
内置一条演示航线 (穿越禁飞区), 便于快速验证。

运行（需 ROS2 Humble 环境）:
    python3 main.py
    # 另一终端发布位置测试:
    # ros2 topic pub /drone/pose geometry_msgs/PoseStamped "{pose: {position: {x: 50.0, y: 0.0, z: 50.0}}}" -1
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import core


class GeofenceNode(Node):

    def __init__(self):
        super().__init__('geofence_detector')
        # 演示禁飞区: 半径 100m 圆柱 + 一个矩形区
        self.detector = core.GeofenceDetector([
            core.CylinderGeofence(center=(0, 0), radius=100, z_max=120,
                                  warning_distance=30),
            core.RectGeofence(300, 400, -50, 50, z_max=80,
                              warning_distance=30),
        ])
        self.sub = self.create_subscription(
            PoseStamped, 'drone/pose', self._on_pose, 10)
        self.pub_status = self.create_publisher(
            String, 'drone/geofence_status', 10)

        self.ok_count = 0
        self.last_status = None
        self.get_logger().info('电子围栏节点启动, 订阅 /drone/pose')

        # 可选自动退出 (演示/CI 用): exit_after 秒后退出, 0 = 持续运行
        self.declare_parameter('exit_after', 0.0)
        exit_after = float(self.get_parameter('exit_after').value)
        if exit_after > 0:
            self.create_timer(exit_after, lambda: (_ for _ in ()).throw(
                SystemExit(0)))

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        status, dist = self.detector.check(p.x, p.y, p.z)

        out = String()
        out.data = f'{status}:{dist:.1f}'
        self.pub_status.publish(out)

        if status == core.INSIDE:
            self.get_logger().error(
                f'【告急】已进入禁飞区! 深入 {abs(dist):.1f} m, '
                f'请立即脱离 (位置: {p.x:.0f},{p.y:.0f},{p.z:.0f})')
        elif status == core.WARNING:
            self.get_logger().warn(
                f'【预警】距禁飞区边界仅 {dist:.1f} m, 注意规避')
        else:
            self.ok_count += 1
            if self.ok_count % 10 == 0:
                self.get_logger().info(
                    f'围栏状态正常 (最近边界 {dist:.0f} m)')


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
