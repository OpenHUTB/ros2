#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ned_enu_transform/main.py - ROS2 节点入口

演示 NED -> ENU 位姿转换：把一组 OpenHUTB/AirSim NED 航点转换为 ROS ENU 位姿，
发布到 /drone/pose_enu (geometry_msgs/PoseStamped)，同时在终端打印对照表。

运行（需 ROS2 Humble 环境）:
    python3 main.py
验证:
    另一个终端: ros2 topic echo /drone/pose_enu
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import core

# 演示航点（NED 坐标, 米）: 模拟无人机方形航线（z 为负表示飞行高度）
DEMO_NED_WAYPOINTS = [
    # (x 北, y 东, z 下, yaw_ned rad)
    (0.0, 0.0, -10.0, 0.0),
    (50.0, 0.0, -10.0, 0.0),
    (50.0, 30.0, -10.0, 1.5708),
    (0.0, 30.0, -10.0, 3.1416),
    (0.0, 0.0, -10.0, 4.7124),
]


class NedEnuDemoNode(Node):

    def __init__(self):
        super().__init__('ned_enu_demo')
        self.pub = self.create_publisher(PoseStamped, 'drone/pose_enu', 10)
        self.waypoints = list(DEMO_NED_WAYPOINTS)
        self.index = 0

        self.get_logger().info('NED->ENU 转换演示节点启动, 话题: /drone/pose_enu')
        self._print_table()
        self.timer = self.create_timer(0.5, self._publish_next)

    def _print_table(self):
        self.get_logger().info(
            'NED(北,东,下,偏航) -> ENU(x,y,z,偏航) 对照表:')
        for x_n, y_n, z_n, yaw_n in self.waypoints:
            x_e, y_e, z_e = core.ned_to_enu_position(x_n, y_n, z_n)
            yaw_e = core.ned_to_enu_yaw(yaw_n)
            self.get_logger().info(
                f'  NED({x_n:6.1f},{y_n:6.1f},{z_n:6.1f},{yaw_n:6.3f}) -> '
                f'ENU({x_e:6.1f},{y_e:6.1f},{z_e:6.1f},{yaw_e:6.3f})')

    def _publish_next(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info('演示航点发布完毕, 节点退出')
            raise SystemExit(0)

        x_n, y_n, z_n, yaw_n = self.waypoints[self.index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        x_e, y_e, z_e = core.ned_to_enu_position(x_n, y_n, z_n)
        yaw_e = core.ned_to_enu_yaw(yaw_n)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x_e, y_e, z_e
        q = core.yaw_to_quaternion_enu(yaw_e)
        msg.pose.orientation.w, msg.pose.orientation.x = q[0], q[1]
        msg.pose.orientation.y, msg.pose.orientation.z = q[2], q[3]

        self.pub.publish(msg)
        self.get_logger().info(
            f'[{self.index + 1}/{len(self.waypoints)}] 发布 ENU 位姿 '
            f'({x_e:.1f}, {y_e:.1f}, {z_e:.1f})')
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = NedEnuDemoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
