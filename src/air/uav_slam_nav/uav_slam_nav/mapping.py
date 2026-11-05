# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""占据栅格建图节点。

订阅 ``/uav/depth`` 与 ``/uav/pose``，用针孔模型把深度图反投影成相机系点云，
再经外参旋转/平移变换到世界系，投影到二维栅格并用 log-odds 更新占据概率，
最后以 ``nav_msgs/OccupancyGrid`` 发布到 ``/map``。

* 自由空间用 Bresenham 射线追踪标记，避免只标记端点导致地图“只有墙没有空地”。
* 所有阈值都用 log-odds 累加，观测多次才判定占据，单帧噪声不会污染地图。
"""

import math
import os

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster

from uav_slam_nav.grid_utils import (
    bresenham,
    camera_intrinsics,
    quat_to_rot_matrix,
    world_to_grid,
)


class Mapping(Node):
    """深度相机 -> 二维占据栅格。"""

    def __init__(self):
        super().__init__('mapping', namespace='uav')

        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('map_size', 50.0)
        self.declare_parameter('max_depth', 15.0)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('min_height', 0.2)
        self.declare_parameter('max_height', 6.0)
        self.declare_parameter('hfov', 90.0)
        self.declare_parameter('pixel_step', 6)
        self.declare_parameter('max_rays', 1200)
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('log_odds_occupied', 0.85)
        self.declare_parameter('log_odds_free', 0.40)
        self.declare_parameter('log_odds_min', -5.0)
        self.declare_parameter('log_odds_max', 5.0)
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.20)
        self.declare_parameter('pose_frame_id', 'airsim_ned')
        self.declare_parameter('camera_offset_x', 0.0)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('camera_offset_z', 0.0)
        self.declare_parameter('output_dir', '~/.ros/uav_slam_nav')
        self.declare_parameter('save_map', False)

        self._res = float(self.get_parameter('resolution').value)
        self._size = float(self.get_parameter('map_size').value)
        self._max_depth = float(self.get_parameter('max_depth').value)
        self._min_depth = float(self.get_parameter('min_depth').value)
        self._min_height = float(self.get_parameter('min_height').value)
        self._max_height = float(self.get_parameter('max_height').value)
        self._hfov = float(self.get_parameter('hfov').value)
        self._pixel_step = max(1, int(self.get_parameter('pixel_step').value))
        self._max_rays = max(1, int(self.get_parameter('max_rays').value))
        self._update_rate = max(0.2, float(self.get_parameter('update_rate').value))
        self._publish_rate = max(0.2, float(self.get_parameter('publish_rate').value))
        p_occ = self._p_occ_param('log_odds_occupied')
        self._l_occ = math.log(p_occ / (1.0 - p_occ))
        p_free = self._p_occ_param('log_odds_free')
        self._l_free = math.log(p_free / (1.0 - p_free))
        self._l_min = float(self.get_parameter('log_odds_min').value)
        self._l_max = float(self.get_parameter('log_odds_max').value)
        self._occ_thresh = float(self.get_parameter('occupied_thresh').value)
        self._free_thresh = float(self.get_parameter('free_thresh').value)
        self._frame = str(self.get_parameter('pose_frame_id').value)
        self._offset = np.array(
            [
                float(self.get_parameter('camera_offset_x').value),
                float(self.get_parameter('camera_offset_y').value),
                float(self.get_parameter('camera_offset_z').value),
            ],
            dtype=np.float64,
        )
        self._output_dir = str(self.get_parameter('output_dir').value)

        self._n = int(round(self._size / self._res))
        self._origin_x = -self._size / 2.0
        self._origin_y = -self._size / 2.0
        self._log_odds = np.zeros((self._n, self._n), dtype=np.float32)

        self._latest_depth = None
        self._latest_pose = None
        self._updates = 0

        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.create_subscription(Image, 'depth', self._on_depth, 5)
        self.create_subscription(PoseStamped, 'pose', self._on_pose, 10)

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        self.create_timer(1.0 / self._update_rate, self._integrate)
        self.create_timer(1.0 / self._publish_rate, self._publish_map)

        self.get_logger().info(
            f'建图节点启动：{self._n}x{self._n} 栅格，分辨率 {self._res} m，'
            f'范围 {self._size} m，更新 {self._update_rate:.1f} Hz')

    # ------------------------------------------------------------------ 工具
    def _p_occ_param(self, name):
        """读取并裁剪概率参数，保证 log-odds 计算不会除零或发散。"""
        value = float(self.get_parameter(name).value)
        return min(0.99, max(0.01, value))

    def _publish_static_transforms(self):
        """发布 map->odom、odom->airsim_ned 的单位静态变换。

        本模块不做位姿估计，``/uav/pose`` 本身就是 AirSim 的世界系 NED 位姿，
        所以这三帧在二维平面内重合；显式发出来只是为了让 rviz 的 TF 树完整。
        """
        transforms = []
        for parent, child in (('map', 'odom'), ('odom', self._frame)):
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.rotation.w = 1.0
            transforms.append(msg)
        self._tf_broadcaster.sendTransform(transforms)

    # ------------------------------------------------------------------ 订阅
    def _on_depth(self, msg):
        """缓存一帧深度图与相机内参，真正的积分放到定时器里做。"""
        if msg.encoding != '32FC1':
            self.get_logger().warn(f'深度图编码应为 32FC1，收到 {msg.encoding}', once=True)
            return
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            return
        depth = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if depth.size != width * height:
            return
        fx, fy, cx, cy = camera_intrinsics(width, height, self._hfov)
        self._latest_depth = (depth.reshape(height, width), width, height, fx, fy, cx, cy)

    def _on_pose(self, msg):
        """缓存当前位姿，位姿只用来做点云变换和射线起点。"""
        quat = msg.pose.orientation
        position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        rotation = quat_to_rot_matrix(quat.x, quat.y, quat.z, quat.w)
        self._latest_pose = (position, rotation)

    # ------------------------------------------------------------------ 积分
    def _integrate(self):
        """把最新一帧深度图积分进栅格：端点标占据，射线上其余格子标自由。"""
        if self._latest_depth is None or self._latest_pose is None:
            return

        depth, width, height, fx, fy, cx, cy = self._latest_depth
        position, rotation = self._latest_pose

        step = self._pixel_step
        cols = np.arange(0, width, step)
        rows = np.arange(0, height, step)
        grid_u, grid_v = np.meshgrid(cols, rows)
        sampled = depth[np.ix_(rows, cols)]

        valid = (
            np.isfinite(sampled)
            & (sampled > self._min_depth)
            & (sampled < self._max_depth)
        )
        if not np.any(valid):
            return

        u = grid_u[valid].astype(np.float64)
        v = grid_v[valid].astype(np.float64)
        z = sampled[valid].astype(np.float64)

        # 针孔模型：像素 -> 相机系
        cam_x = (u - cx) * z / fx
        cam_y = (v - cy) * z / fy
        cam_z = z

        # 相机系 -> 机体系。AirSim 前视相机光轴指向机体 +x：
        #   机体 x = 相机 z，机体 y = 相机 x，机体 z = 相机 y
        body = np.stack([cam_z, cam_x, cam_y]) + self._offset.reshape(3, 1)

        # 机体系 -> 世界系
        world = rotation @ body + position.reshape(3, 1)
        heights = -world[2]  # NED 的 z 向下，取负得到“离地高度”

        keep = (heights >= self._min_height) & (heights <= self._max_height)
        if not np.any(keep):
            return
        world = world[:, keep]

        gx = np.floor((world[0] - self._origin_x) / self._res).astype(np.int64)
        gy = np.floor((world[1] - self._origin_y) / self._res).astype(np.int64)
        inside = (gx >= 0) & (gx < self._n) & (gy >= 0) & (gy < self._n)
        if not np.any(inside):
            return
        gx = gx[inside]
        gy = gy[inside]

        if gx.size > self._max_rays:
            pick = np.linspace(0, gx.size - 1, self._max_rays).astype(np.int64)
            gx = gx[pick]
            gy = gy[pick]

        origin_gx, origin_gy = world_to_grid(
            position[0], position[1], self._origin_x, self._origin_y, self._res)

        n = self._n
        free_index = []
        occupied_index = []
        for cell_x, cell_y in zip(gx, gy):
            cells = bresenham(origin_gx, origin_gy, int(cell_x), int(cell_y))
            for cx_i, cy_i in cells[:-1]:
                if 0 <= cx_i < n and 0 <= cy_i < n:
                    free_index.append(cy_i * n + cx_i)
            last_x, last_y = cells[-1]
            if 0 <= last_x < n and 0 <= last_y < n:
                occupied_index.append(last_y * n + last_x)

        if not free_index and not occupied_index:
            return

        delta = np.zeros(n * n, dtype=np.float32)
        if free_index:
            np.add.at(delta, np.asarray(free_index, dtype=np.int64), self._l_free)
        if occupied_index:
            np.add.at(delta, np.asarray(occupied_index, dtype=np.int64), self._l_occ)

        self._log_odds += delta.reshape(n, n)
        np.clip(self._log_odds, self._l_min, self._l_max, out=self._log_odds)

        self._updates += 1
        if self._updates % 20 == 0:
            known = int(np.count_nonzero(np.abs(self._log_odds) > 1e-3))
            self.get_logger().info(
                f'已积分 {self._updates} 帧，已知栅格 {known}/{n * n} '
                f'({100.0 * known / (n * n):.1f}%)')

    # ------------------------------------------------------------------ 发布
    def _probability(self):
        """log-odds -> 占据概率，用数值稳定的 sigmoid 写法。"""
        return 1.0 - 1.0 / (1.0 + np.exp(self._log_odds))

    def _publish_map(self):
        """发布 OccupancyGrid：100 占据，0 自由，-1 未知。"""
        probability = self._probability()
        flat = probability.ravel()
        data = np.full(self._n * self._n, -1, dtype=np.int8)
        data[flat > self._occ_thresh] = 100
        data[flat < self._free_thresh] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self._res
        msg.info.width = self._n
        msg.info.height = self._n
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.tolist()
        self._map_pub.publish(msg)

    # -------------------------------------------------------------- 落盘（可选）
    def save_map(self, path=None):
        """把当前栅格保存成 PGM + YAML（运行产物，放在 ~/.ros 下，不进仓库）。"""
        probability = self._probability()
        unknown = np.abs(self._log_odds) < 1e-3
        gray = np.where(probability > self._occ_thresh, 0, 255).astype(np.uint8)
        gray[unknown] = 205
        image = np.flipud(gray)  # PGM 第 0 行在上方，栅格第 0 行在下方
        out_dir = os.path.expanduser(path or self._output_dir)
        os.makedirs(out_dir, exist_ok=True)
        pgm_path = os.path.join(out_dir, 'map.pgm')
        with open(pgm_path, 'wb') as handle:
            header = f'P5\n{self._n} {self._n}\n255\n'.encode('ascii')
            handle.write(header)
            handle.write(image.tobytes())
        yaml_path = os.path.join(out_dir, 'map.yaml')
        with open(yaml_path, 'w', encoding='utf-8') as handle:
            handle.write(f'image: map.pgm\nresolution: {self._res}\n')
            handle.write(f'origin: [{self._origin_x}, {self._origin_y}, 0.0]\n')
            handle.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.20\n')
        self.get_logger().info(f'地图已保存：{pgm_path} 与 {yaml_path}')
        return pgm_path
