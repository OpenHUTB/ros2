#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
turtlebot3_amcl.py —— 高精度定位与重定位（AMCL 式粒子滤波）· Gazebo/TurtleBot3 渲染版
====================================================================================
在 Gazebo + TurtleBot3 仿真中，仅靠激光雷达 /scan 与里程计 /odom 做『全局定位 /
重定位』：当机器人不知道自己在地图上的位置时，粒子在整张地图上均匀撒开（全局搜索），
随着机器人移动与激光观测，粒子滤波收敛到真实位姿，并持续修正里程计漂移，
输出高精度 /amcl_pose 供导航（Nav2）使用。

核心算法（栅格地图距离场 + likelihood field 权重 + 低方差重采样粒子滤波）
与 2D 轻量版 sim_2d_relocalization.py 完全共用 reloc_core.ParticleFilter，
本文件只做『ROS2 数据接入 + 地图构建 + 话题收发』。

使用前提（本机 Gazebo，本仓库不提供运行环境）:
  1. 启动 TurtleBot3 世界仿真，并加载一张栅格地图
     （map_server 发布 /map，类型 nav_msgs/OccupancyGrid）
  2. 机器人在世界中移动，发布 /scan(sensor_msgs/LaserScan) 与 /odom(nav_msgs/Odometry)
运行:
  python3 turtlebot3_amcl.py
输出:
  /amcl_pose          geometry_msgs/PoseWithCovarianceStamped   高精度位姿
  /particle_cloud     geometry_msgs/PoseArray                  粒子云（可 RViz 可视化）

说明：本项目为课程/学习用途的仿真代码，不含真实道路行驶与真车控制，遵循开源许可。
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from reloc_core import GridMap, ParticleFilter


# ============================================================ OccupancyGrid → GridMap
def occupancy_grid_to_map(grid):
    """把 nav_msgs/OccupancyGrid 转成 reloc_core.GridMap（占据单元作为障碍矩形）。
    返回 (地图, 世界坐标范围) 。
    """
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    w, h = grid.info.width, grid.info.height
    x0, x1 = ox, ox + w * res
    y0, y1 = oy, oy + h * res
    m = GridMap(max(x1 - x0, y1 - y0), max(res, 0.1))
    margin = 0.05
    # 外边界（四墙）
    m.add_rect_obstacle(x0 - margin, y0 - margin, x1 + margin, y1 + margin)
    # 占据单元（occupancy >= 50 视为障碍）
    step = max(1, int(round(res / 0.1)))          # 粗采样，避免单元过多拖慢距离场
    for j in range(0, h, step):
        for i in range(0, w, step):
            v = grid.data[j * w + i]
            if v is None or v < 0 or v < 50:
                continue
            cx = ox + (i + 0.5) * res
            cy = oy + (j + 0.5) * res
            m.add_rect_obstacle(cx - res / 2, cy - res / 2, cx + res / 2, cy + res / 2)
    m.build_distance_field()
    return m, (x0, y0, x1, y1)


# ============================================================ ROS2 重定位节点
class TurtleBot3AMCL(Node):
    def __init__(self):
        super().__init__('turtlebot3_amcl')
        self.declare_parameter('n_particles', 1000)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')
        self.n_particles = int(self.get_parameter('n_particles').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.map = None            # GridMap
        self.map_ready = False
        self.angles = None
        self.pf = None             # ParticleFilter
        self.odom_pose = None      # 里程计航位推算位姿（含漂移）
        self.prev_odom = None
        self._map_lock = threading.Lock()

        # 订阅地图 / 里程计 / 激光
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                 self.on_map, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.on_odom, 100)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, 100)
        # 发布重定位结果
        self.amcl_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        self.get_logger().info(
            f"TurtleBot3 AMCL 节点就绪（粒子数 {self.n_particles}，等待地图 / 里程计 / 激光）")

    # ---- 地图：收到一次即完成地图构建，粒子在全图均匀初始化 ----
    def on_map(self, msg):
        with self._map_lock:
            if self.map_ready:
                return
            self.map, (x0, y0, x1, y1) = occupancy_grid_to_map(msg)
            L = max(x1 - x0, y1 - y0)
            self.map_ready = True
            # 初始位姿未知 → 粒子在整个方形地图范围均匀撒开（全局重定位）
            rng = np.random.default_rng(0)
            self.pf = ParticleFilter(self.n_particles, rng, bounds=(x0, x0 + L))
            self.pf.init_uniform()
            self.odom_pose = None
            self.prev_odom = None
            self.get_logger().info(
                f"地图已加载 {msg.info.width}x{msg.info.height}@{msg.info.resolution}，"
                f"粒子在全图均匀初始化 → 进入全局重定位")

    # ---- 里程计：累计航位推算（含漂移，用于对比），并给粒子运动模型提供增量 ----
    def on_odom(self, msg):
        if not self.map_ready:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        th = 2.0 * math.atan2(q.z, q.w)                       # yaw
        cur = np.array([p.x, p.y, th])
        if self.prev_odom is None:
            self.prev_odom = cur
            self.odom_pose = cur.copy()
            return
        dpos = cur[:2] - self.prev_odom[:2]                   # 里程计位移
        dtheta = (cur[2] - self.prev_odom[2] + math.pi) % (2 * math.pi) - math.pi
        self.prev_odom = cur
        # 粒子按里程计增量推进（AMCL 运动模型）
        with self._map_lock:
            if self.pf is not None:
                self.pf.predict(dpos, dtheta)
        # 航位推算（用于对比展示漂移）
        d = float(np.linalg.norm(dpos))
        self.odom_pose[2] += dtheta + 0.010                   # 模拟陀螺漂移
        self.odom_pose[2] = (self.odom_pose[2] + math.pi) % (2 * math.pi) - math.pi
        self.odom_pose[:2] += d * 1.04 * np.array([math.cos(self.odom_pose[2]),
                                                   math.sin(self.odom_pose[2])])

    # ---- 激光：观测更新 + 低方差重采样，发布 amcl_pose 与粒子云 ----
    def on_scan(self, msg):
        if not self.map_ready or self.pf is None:
            return
        if len(msg.ranges) == 0:
            return
        # 构造等角分辨率角度数组（TurtleBot3 LaserScan 通常是 0..2π）
        n = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_min + n * msg.angle_increment,
                             n, endpoint=False)
        ranges = np.clip(np.array(msg.ranges, dtype=float),
                         0.02, self.max_range)
        with self._map_lock:
            self.pf.update(self.map, ranges, angles, max_range=self.max_range)
            self.pf.resample()
            est = self.pf.estimate()
            spread = float(np.sqrt(np.var(self.pf.particles[:, 0])
                                   + np.var(self.pf.particles[:, 1])))
        # 发布高精度位姿
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.pose.pose.position.x = float(est[0])
        out.pose.pose.position.y = float(est[1])
        out.pose.pose.orientation.z = math.sin(est[2] / 2.0)
        out.pose.pose.orientation.w = math.cos(est[2] / 2.0)
        out.pose.covariance[0] = spread * spread              # x 方差
        out.pose.covariance[7] = spread * spread              # y 方差
        self.amcl_pub.publish(out)
        # 发布粒子云（RViz 可视化）
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'
        for px, py, pth in self.pf.particles:
            pose = Pose()
            pose.position.x = float(px)
            pose.position.y = float(py)
            pose.orientation.z = math.sin(pth / 2.0)
            pose.orientation.w = math.cos(pth / 2.0)
            pa.poses.append(pose)
        self.particles_pub.publish(pa)
        if spread < 0.35:
            self.get_logger().info(
                f"[relocalized] spread={spread:.2f} m  pose=({est[0]:.2f},{est[1]:.2f},{math.degrees(est[2]):.0f}°)",
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3AMCL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
