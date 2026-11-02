"""ROS2 节点：订阅激光与定位，发布速度指令。

仅在安装了 rclpy 的环境中可用；未安装时导入本模块会抛出 ImportError，
离线仿真请直接使用 sim/run_sim.py。

话题约定：
    订阅 /scan    (sensor_msgs/LaserScan)
    订阅 /odom    (nav_msgs/Odometry)
    发布 /cmd_vel (geometry_msgs/Twist)
    发布 /pp_info (std_msgs/String，JSON 调试信息)
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .arbitrate import arbitrate
from .obstacle import ObstacleAvoider
from .paths import Path, load_path
from .pure_pursuit import PurePursuitController
from .robot import Pose


class PurePursuitNode(Node):
    def __init__(self) -> None:
        super().__init__("pure_pursuit_node")

        self.declare_parameter("target_speed", 0.8)
        self.declare_parameter("path_file", "")
        self.declare_parameter("enable_avoidance", True)

        self.target_speed = float(self.get_parameter("target_speed").value)
        path_file = str(self.get_parameter("path_file").value)
        self.enable_avoidance = bool(self.get_parameter("enable_avoidance").value)

        if path_file:
            self.path = load_path(path_file)
        else:
            from .paths import circle

            self.path = circle(radius=8.0)
        self.get_logger().info(f"载入参考路径，共 {len(self.path)} 个点")

        self.controller = PurePursuitController()
        self.avoider = ObstacleAvoider()
        self.pose = Pose()
        self.scan = None

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.info_pub = self.create_publisher(String, "/pp_info", 10)
        self.create_timer(0.05, self._on_timer)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # 四元数转 yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose = Pose(x=p.x, y=p.y, theta=math.atan2(siny, cosy))

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = (np.asarray(msg.ranges, dtype=float), msg.angle_min, msg.angle_increment)

    def _publish_cmd_vel(self, v: float, w: float) -> None:
        """统一出口：无论由哪条分支得出速度，都在此处发布。"""
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def _on_timer(self) -> None:
        # 先解算一次纯追踪，拿到本周期的状态量（含是否到达终点）。
        # 这里只是取状态，尚未决定是否采纳其输出。
        v_pp, w_pp, info = self.controller.compute(self.path, self.pose, self.target_speed)

        # 避障建议；未开启避障或本周期无激光数据时为 None
        av = None
        if self.enable_avoidance and self.scan is not None:
            ranges, amin, ainc = self.scan
            av = self.avoider.evaluate(ranges, amin, ainc)

        # 按优先级裁决最终速度：终点停车 > 纯追踪 > 避障覆盖。
        # 裁决逻辑抽在 arbitrate() 中，不依赖 rclpy，因此可被单元测试覆盖。
        res = arbitrate(v_pp, w_pp, info.finished, av)
        if res.reason == "finished":
            self.get_logger().info("已到达终点，停车。", throttle_duration_sec=2.0)

        # 统一出口发布（终点处发布零速，保证持续刹停）
        self._publish_cmd_vel(res.v, res.w)

        payload = {
            "cte": info.cte,
            "alpha": info.alpha,
            "curvature": info.curvature,
            "lookahead": info.lookahead,
            "v": res.v,
            "w": res.w,
            "reason": res.reason,
        }
        self.info_pub.publish(String(data=json.dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
