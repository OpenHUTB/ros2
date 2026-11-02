#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_trajectory_planner/main.py - ROS2 节点入口

把规划好的航点轨迹发布为 nav_msgs/Path（ENU），供 RViz 或其他节点消费；
同时在终端打印规划摘要。

运行（需 ROS2 Humble 环境）:
    python3 main.py
验证:
    另一个终端: ros2 topic echo /drone/planned_path --once
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import core


def ned_to_enu_position(x, y, z):
    """NED -> ENU 位置转换 (y、z 取反), 与 ned_enu_transform 模块同一规则。"""
    return (x, -y, -z)

# 演示航线（NED）: 巡检方形航线, 高度 10m
DEMO_WAYPOINTS_NED = [
    (0.0, 0.0, -10.0),
    (0.0, 50.0, -10.0),
    (50.0, 50.0, -10.0),
    (50.0, 0.0, -10.0),
    (0.0, 0.0, -10.0),
]


class WaypointPlannerNode(Node):

    def __init__(self):
        super().__init__('waypoint_trajectory_planner')
        self.pub = self.create_publisher(Path, 'drone/planned_path', 10)

        self.traj = core.WaypointTrajectory(
            DEMO_WAYPOINTS_NED, cruise_speed=5.0, hover_time=2.0, sample_dt=0.5)

        self.get_logger().info(
            f'轨迹规划完成: {len(DEMO_WAYPOINTS_NED)} 个航点, '
            f'总航程 {self.traj.total_length:.1f} m, '
            f'总时长 {self.traj.total_time:.1f} s (含悬停)')

        # 发布完整轨迹 (NED -> ENU)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'world'
        for t, (x_n, y_n, z_n) in self.traj.sample()[::10]:  # 每 5s 一个点
            x_e, y_e, z_e = ned_to_enu_position(x_n, y_n, z_n)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = \
                x_e, y_e, z_e
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.pub.publish(path)
        self.get_logger().info(
            f'已发布 {len(path.poses)} 个路径点到 /drone/planned_path')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
