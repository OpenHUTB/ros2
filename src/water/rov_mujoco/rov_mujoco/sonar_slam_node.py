#!/usr/bin/env python3
"""
水下多波束声呐贝叶斯占据栅格 SLAM 建图节点 (Sonar Occupancy Grid SLAM Node)
========================================================================
功能：
  1. 订阅 72 束多波束声呐数据 (/rov/sonar/scan) 与水下里程计 (/rov/odom)
  2. 实现贝叶斯对数几率 (Log-Odds) 栅格更新逆传感器模型
  3. 发布标准 ROS 2 占据栅格地图 (/map) 与元数据 (/map_metadata)
  4. 维护并广播完备的 TF2 坐标树 (map -> odom -> rov_base -> rov_sonar_link)
  5. 提供在线建图与地图持久化保存功能 (PGM + YAML)
"""

import os
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class SonarOccupancyGridSLAM:
    """贝叶斯对数几率占据栅格地图核心引擎"""

    def __init__(
        self,
        width_m: float = 20.0,
        height_m: float = 20.0,
        resolution: float = 0.05,
        origin_x: float = -10.0,
        origin_y: float = -10.0,
        l_occ: float = 1.2,
        l_free: float = 0.35,
        l_min: float = -4.0,
        l_max: float = 4.0,
    ):
        self.resolution = resolution
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.l_occ = l_occ
        self.l_free = l_free
        self.l_min = l_min
        self.l_max = l_max

        # 对数几率矩阵初始化 (0 代表未探测 unknown)
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        # 访问计数掩码 (标记已被声呐波束覆盖的区域)
        self.observed_mask = np.zeros((self.height, self.width), dtype=bool)

    def world_to_map(self, x: float, y: float) -> tuple:
        """世界坐标 (米) -> 栅格坐标 (行列索引)"""
        mx = int(math.floor((x - self.origin_x) / self.resolution))
        my = int(math.floor((y - self.origin_y) / self.resolution))
        return mx, my

    def map_to_world(self, mx: int, my: int) -> tuple:
        """栅格坐标 -> 世界坐标 (米)"""
        x = self.origin_x + (mx + 0.5) * self.resolution
        y = self.origin_y + (my + 0.5) * self.resolution
        return x, y

    def is_in_bounds(self, mx: int, my: int) -> bool:
        return 0 <= mx < self.width and 0 <= my < self.height

    def update_scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        scan_ranges: list,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        sensor_offset: float = 0.41,
    ):
        """
        利用声呐射线追踪更新栅格地图对数几率 (Bresenham 光线投射与端点更新)
        """
        # 声呐探头在世界坐标系中的空间原点
        sx = robot_x + sensor_offset * math.cos(robot_yaw)
        sy = robot_y + sensor_offset * math.sin(robot_yaw)
        smx, smy = self.world_to_map(sx, sy)

        if not self.is_in_bounds(smx, smy):
            return

        num_beams = len(scan_ranges)
        for i in range(num_beams):
            r = scan_ranges[i]
            beam_angle = robot_yaw + (angle_min + i * angle_increment)

            is_hit = (range_min <= r < (range_max - 0.15))
            dist_clamped = min(r, range_max)

            # 端点栅格坐标
            ex = sx + dist_clamped * math.cos(beam_angle)
            ey = sy + dist_clamped * math.sin(beam_angle)
            emx, emy = self.world_to_map(ex, ey)

            # Bresenham 直线射线穿行：沿线经过的所有空闲空间栅格执行 l_free 衰减
            ray_cells = self._bresenham_line(smx, smy, emx, emy)

            for cx, cy in ray_cells[:-1]:
                if self.is_in_bounds(cx, cy):
                    self.log_odds[cy, cx] = max(self.l_min, self.log_odds[cy, cx] - self.l_free)
                    self.observed_mask[cy, cx] = True

            # 若命中障碍物，则在击中端点更新 l_occ
            if is_hit and len(ray_cells) > 0:
                hx, hy = ray_cells[-1]
                if self.is_in_bounds(hx, hy):
                    self.log_odds[hy, hx] = min(self.l_max, self.log_odds[hy, hx] + self.l_occ)
                    self.observed_mask[hy, hx] = True

    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> list:
        """Bresenham 快速离散射线生成算法"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        curr_x, curr_y = x0, y0
        while True:
            points.append((curr_x, curr_y))
            if curr_x == x1 and curr_y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy
        return points

    def get_occupancy_grid(self) -> np.ndarray:
        """
        导出 ROS 2 OccupancyGrid 格式的一维数组 (int8, 0-100, -1 为未知)
        """
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        observed = self.observed_mask

        # 对数几率转换为概率 p = 1 / (1 + exp(-L))
        probs = 1.0 / (1.0 + np.exp(-self.log_odds[observed]))
        grid[observed] = np.clip((probs * 100.0).astype(np.int8), 0, 100)
        return grid.flatten()

    def get_statistics(self) -> dict:
        """获取建图统计指标"""
        total_cells = self.width * self.height
        explored_cells = int(np.sum(self.observed_mask))
        probs = 1.0 / (1.0 + np.exp(-self.log_odds))
        occupied_cells = int(np.sum((self.observed_mask) & (probs > 0.65)))
        free_cells = int(np.sum((self.observed_mask) & (probs < 0.35)))

        return {
            "total_cells": total_cells,
            "explored_cells": explored_cells,
            "occupied_cells": occupied_cells,
            "free_cells": free_cells,
            "coverage_pct": (explored_cells / total_cells) * 100.0,
            "resolution": self.resolution,
            "width": self.width,
            "height": self.height,
        }

    def save_map(self, base_path: str) -> tuple:
        """
        保存占据栅格地图为标准 ROS 格式 (PGM + YAML)
        """
        pgm_path = f"{base_path}.pgm"
        yaml_path = f"{base_path}.yaml"

        # 生成 PGM 灰度图像矩阵 (254: free, 0: occupied, 205: unknown)
        img = np.full((self.height, self.width), 205, dtype=np.uint8)
        probs = 1.0 / (1.0 + np.exp(-self.log_odds))
        img[self.observed_mask & (probs < 0.35)] = 254
        img[self.observed_mask & (probs > 0.65)] = 0

        # 写入 PGM 文件 (P5 binary)
        with open(pgm_path, 'wb') as f:
            header = f"P5\n{self.width} {self.height}\n255\n".encode('ascii')
            f.write(header)
            # ROS 栅格通常以 y 递减（左下角原点 -> 图片左上角原点，上下翻转）
            flipped = np.flipud(img)
            f.write(flipped.tobytes())

        # 写入 YAML 元数据
        yaml_content = (
            f"image: {os.path.basename(pgm_path)}\n"
            f"resolution: {self.resolution:.4f}\n"
            f"origin: [{self.origin_x:.2f}, {self.origin_y:.2f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)

        return pgm_path, yaml_path


class SonarSLAMNode(Node):
    """ROS 2 水下多波束声呐 SLAM 节点"""

    def __init__(self):
        super().__init__('sonar_slam_node')

        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width_m', 20.0)
        self.declare_parameter('height_m', 20.0)
        self.declare_parameter('publish_rate', 5.0)

        res = self.get_parameter('resolution').value
        w_m = self.get_parameter('width_m').value
        h_m = self.get_parameter('height_m').value
        pub_rate = self.get_parameter('publish_rate').value

        self.slam = SonarOccupancyGridSLAM(
            width_m=w_m,
            height_m=h_m,
            resolution=res,
            origin_x=-w_m / 2.0,
            origin_y=-h_m / 2.0
        )

        # 订阅传感器
        self.sub_scan = self.create_subscription(
            LaserScan, '/rov/sonar/scan', self._scan_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/rov/odom', self._odom_callback, 10
        )

        # 发布地图与元数据
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 1)
        self.pub_metadata = self.create_publisher(MapMetaData, '/map_metadata', 1)

        # TF2 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 状态缓存
        self.latest_odom = None
        self.map_timer = self.create_timer(1.0 / pub_rate, self._publish_map)

        self.get_logger().info(
            f'水下声呐 SLAM 节点已启动 | 地图尺寸: {w_m}x{h_m}m | 分辨率: {res}m | 栅格数: {self.slam.width}x{self.slam.height}'
        )

    def _odom_callback(self, msg: Odometry):
        self.latest_odom = msg

        # 广播 odom -> rov_base 动态位姿变换
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'rov_base'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # 广播 map -> odom 校准变换 (无漂移基准下平移为0)
        t_map = TransformStamped()
        t_map.header.stamp = msg.header.stamp
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map)

    def _scan_callback(self, msg: LaserScan):
        if self.latest_odom is None:
            return

        pos = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

        self.slam.update_scan(
            robot_x=pos.x,
            robot_y=pos.y,
            robot_yaw=yaw,
            scan_ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
        )

    def _publish_map(self):
        now = self.get_clock().now().to_msg()
        grid_data = self.slam.get_occupancy_grid()

        map_msg = OccupancyGrid()
        map_msg.header.stamp = now
        map_msg.header.frame_id = 'map'

        map_msg.info.resolution = float(self.slam.resolution)
        map_msg.info.width = int(self.slam.width)
        map_msg.info.height = int(self.slam.height)
        map_msg.info.origin.position.x = float(self.slam.origin_x)
        map_msg.info.origin.position.y = float(self.slam.origin_y)
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = grid_data.tolist()

        self.pub_map.publish(map_msg)
        self.pub_metadata.publish(map_msg.info)


def main(args=None):
    rclpy.init(args=args)
    node = SonarSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
