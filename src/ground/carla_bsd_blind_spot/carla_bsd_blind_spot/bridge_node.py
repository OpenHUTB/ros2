#!/usr/bin/env python3
"""CARLA↔ROS 桥接节点（BSD 版）。

职责：
  * 连接 CARLA（同步模式 20 Hz），清理残留 actor，生成自车；
  * 挂顶置前向 LiDAR，按左右侧别窗口 + 车身高度带 + 径向聚类测距
    （8 帧时间融合抗稀疏闪烁，与 AEB 模块实测结论一致）；
  * 发布 /bsd/left_distance、/bsd/right_distance（Float32，米，无目标=valid_max）
    与 /bsd/ego_speed（Float32，m/s）；
  * 主循环按固定步长 world.tick()，退出时恢复异步模式并清理。

自车不施加任何控制（BSD 只监测预警，不干预驾驶）。
"""

import os
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# 让 colcon install / 源码直跑两种方式都能找到本包
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from bsd_core import blind_zone_distance, parse_lidar_points  # noqa: E402
import carla_common as cc  # noqa: E402


class BSDBridgeNode(Node):

    def __init__(self):
        super().__init__("bsd_bridge_node")
        self.declare_parameter("host", cc.DEFAULT_HOST)
        self.declare_parameter("port", cc.DEFAULT_PORT)
        self.declare_parameter("town", cc.DEFAULT_MAP)
        self.declare_parameter("blueprint", cc.EGO_BLUEPRINT)
        self.declare_parameter("frames_fuse", 8)
        # 与 bsd_core.blind_zone_distance 一致的盲区窗口参数
        self.declare_parameter("x_min", -25.0)
        self.declare_parameter("x_max", 2.0)
        self.declare_parameter("y_near", 2.9)
        self.declare_parameter("y_far", 4.2)
        self.declare_parameter("min_points", 6)
        self.declare_parameter("valid_max", 40.0)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.town = self.get_parameter("town").value
        self.blueprint = self.get_parameter("blueprint").value
        self.frames = int(self.get_parameter("frames_fuse").value)
        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_near = float(self.get_parameter("y_near").value)
        self.y_far = float(self.get_parameter("y_far").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.valid_max = float(self.get_parameter("valid_max").value)

        self.pub_left = self.create_publisher(Float32, "bsd/left_distance", 10)
        self.pub_right = self.create_publisher(Float32, "bsd/right_distance", 10)
        self.pub_speed = self.create_publisher(Float32, "bsd/ego_speed", 10)

        self._buf = deque(maxlen=self.frames)  # 8 帧点云时间融合缓冲
        self.client, self.world = cc.connect(self.host, self.port, self.town)
        cc.cleanup_actors(self.world)
        self.vehicle, tf = cc.spawn_vehicle(self.world, self.blueprint)
        self.lidar = cc.make_lidar(self.world, self.vehicle, self._on_lidar)
        self.get_logger().info(
            f"自车已生成 ({tf.location.x:.1f}, {tf.location.y:.1f})，"
            f"侧向盲区窗口 x[{self.x_min},{self.x_max}] y±[{self.y_near},{self.y_far}]，"
            f"{self.frames} 帧融合")

    def _on_lidar(self, pc):
        # 复制点云：CARLA 会复用 raw_data 底层内存
        self._buf.append(parse_lidar_points(bytes(pc.raw_data)))

    def _zone_distance(self, side):
        if not self._buf:
            return self.valid_max
        pts = np.vstack(self._buf)
        return blind_zone_distance(
            pts, side=side, x_min=self.x_min, x_max=self.x_max,
            y_near=self.y_near, y_far=self.y_far,
            min_points=self.min_points, valid_max=self.valid_max)

    def spin_once(self):
        """一个同步 tick：测距 → 发布。返回是否仍可继续。"""
        left = self._zone_distance("left")
        right = self._zone_distance("right")
        speed = cc.get_speed(self.vehicle)
        self.pub_left.publish(Float32(data=float(left)))
        self.pub_right.publish(Float32(data=float(right)))
        self.pub_speed.publish(Float32(data=float(speed)))
        self.world.tick()
        return True

    def shutdown(self):
        for a in (self.lidar, self.vehicle):
            try:
                a.destroy()
            except Exception:  # noqa: BLE001
                pass
        cc.restore_async(self.world)

    def main(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                self.spin_once()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BSDBridgeNode()
    try:
        node.main()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
