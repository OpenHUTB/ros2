#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Dubins 路径发布成 nav_msgs/Path，供 RViz 直接查看。

    ros2 run ackermann_dubins_path ros_node.py --ros-args -p goal_yaw:=3.1416
    ros2 topic echo /dubins/path --once
    rviz2   # 添加 Path 显示，Topic 选 /dubins/path

节点启动时规划一次，之后以 1 Hz 重复发布（RViz 后启动也能收到）。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from dubins import dubins_path


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DubinsPathNode(Node):
    def __init__(self):
        super().__init__('dubins_path')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', math.pi)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('step', 0.05)

        # transient_local：晚启动的 RViz 也能拿到最后一条路径
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(Path, '/dubins/path', qos)

        self.path = self._build()
        self.timer = self.create_timer(1.0, self._publish)
        self._publish()

    def _build(self):
        p = self.get_parameter
        start = (p('start_x').value, p('start_y').value, p('start_yaw').value)
        goal = (p('goal_x').value, p('goal_y').value, p('goal_yaw').value)
        radius = p('radius').value
        frame = p('frame_id').value
        step = p('step').value

        dub = dubins_path(start, goal, radius)
        msg = Path()
        msg.header.frame_id = frame
        for x, y, th, _kappa in dub.sample(step=step):
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(th)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)

        self.get_logger().info(
            'Dubins %s: %.4f m, R=%.2f m, %d 个位姿'
            % (dub.case, dub.length, radius, len(msg.poses)))
        return msg

    def _publish(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)


def main(argv=None):
    # --ros-args 交给 rclpy 自己解析，节点参数才能生效
    rclpy.init(args=argv)
    node = DubinsPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
