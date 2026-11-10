#!/usr/bin/env python3
"""ROS 2 推理节点：把训练好的神经网络端到端避障策略部署为实时节点。

订阅前视激光（/scan），用 MLP 预测 (steer, throttle)，差速解算后发布 /cmd_vel。
可选订阅 /odom 取 yaw 作为航向反馈。

注意：本节点依赖 ROS 2（rclpy），仅在 ROS 2 环境（如 Ubuntu Humble）中运行；
Windows/纯 MuJoCo 仿真请使用 main.py 的 --collect/--train/--eval/--demo。
"""
import sys
from pathlib import Path

import numpy as np

# 让包源码可被独立运行（colcon 安装后也可正常导入）
_PKG = Path(__file__).resolve().parent
if str(_PKG.parent) not in sys.path:
    sys.path.insert(0, str(_PKG.parent))

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from mujoco_car_nn_avoidance.config import Config
from mujoco_car_nn_avoidance.model import MLP


class NNAvoidanceNode(Node):
    """基于训练好的 MLP 的端到端避障节点。"""

    def __init__(self):
        super().__init__("mujoco_car_nn_avoidance")
        cfg = Config()
        self._cfg = cfg
        # 权重随包安装到 share/.../results/（见 setup.py data_files）
        share = Path(get_package_share_directory("mujoco_car_nn_avoidance"))
        self._net = MLP.load(share / "results" / "mlp_weights.npz")
        self._lidar_angles = np.deg2rad(np.asarray(cfg.lidar_angles_deg, float))
        self._range_m = cfg.lidar_range_m

        self._scan_sub = self.create_subscription(
            LaserScan, "scan", self._on_scan, 10)
        self._odom_sub = self.create_subscription(
            Odometry, "odom", self._on_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._yaw = 0.0
        self._have_scan = False
        self.get_logger().info("NN avoidance node started, awaiting /scan")

    def _on_odom(self, msg):
        # 从四元数取 yaw（绕 z 轴航向）
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = float(np.arctan2(siny, cosy))

    def _on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = len(self._lidar_angles)
        if ranges.size < n:
            return
        rays = ranges[:n]
        # 未命中/超量程截断
        rays = np.where(np.isfinite(rays), rays, self._range_m)
        rays = np.clip(rays, 0.0, self._range_m)

        x = np.concatenate([
            np.clip(rays / self._range_m, 0.0, 1.0),
            [float(np.clip(self._yaw / np.pi, -1.0, 1.0))],
        ]).astype(np.float32)
        steer, throttle = self._net.predict(x)
        steer = float(steer)
        throttle = float(np.clip(throttle, 0.0, 1.0))

        # 差速解算：linear.x 与 angular.z
        v = self._cfg.v_max * throttle
        w = self._cfg.omega_max * steer
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self._cmd_pub.publish(twist)
        self._have_scan = True


def main(args=None):
    rclpy.init(args=args)
    node = NNAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
