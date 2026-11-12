"""地面坡度估计仿真节点。

在 PyBullet 里搭一块已知倾角的刚性地面，用一组竖直向下的射线模拟
深度相机，得到带噪声的地面点云，再用最小二乘与 RANSAC 各拟合一次平面，
把估计坡度与真值一起写入 CSV。

射线扫描而非真实渲染深度图，是为了让「点云从哪来」这件事可验证：
每个点的命中位置由 rayTestBatch 给出，没有渲染误差参与。
"""

import csv
import os

import numpy as np
import pybullet as p
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .plane_fit import (
    fit_plane_least_squares,
    fit_plane_ransac,
    inlier_ratio,
    plane_height,
    slope_angle,
)

CAMERA_HEIGHT = 1.5
PLANE_HALF_EXTENT = 5.0
RAY_FLOOR = -2.0


class SlopeScanNode(Node):
    """倾斜地面 + 射线点云 + 平面拟合，一次采样写一行结果。"""

    def __init__(self):
        super().__init__('slope_sim')
        self.declare_parameter('slope_deg', 10.0)
        self.declare_parameter('noise_std', 0.0)
        self.declare_parameter('grid_n', 40)
        self.declare_parameter('out_dir', 'results')
        self.declare_parameter('seed', 0)
        self.declare_parameter('ransac_threshold', 0.02)
        self.declare_parameter('publish_count', 20)
        self.declare_parameter('publish_rate', 10.0)

        self.slope_deg = float(self.get_parameter('slope_deg').value)
        self.noise_std = float(self.get_parameter('noise_std').value)
        self.grid_n = int(self.get_parameter('grid_n').value)
        self.out_dir = self.get_parameter('out_dir').value
        self.seed = int(self.get_parameter('seed').value)
        self.ransac_threshold = float(
            self.get_parameter('ransac_threshold').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.client = p.connect(p.DIRECT)
        if self.client < 0:
            raise RuntimeError('无法连接 PyBullet 仿真后端')
        self.plane_id = self._build_ground()
        self.publisher = self.create_publisher(
            Float64MultiArray, 'slope_estimate', 10)

    def _build_ground(self):
        """建一块绕 y 轴转过 slope_deg 的板，上表面中心精确过原点。

        PyBullet 的位姿旋转绕模拟体中心进行，若直接设 basePosition=[0,0,-h]，
        倾斜后上表面中心会平移到 -R·(0,0,h)，给高程估计引入 0.6 cm 量级的
        系统偏差（20° 时）。这里把基座反向偏移，保证真值高度恒为 0。
        """
        half_thickness = 0.1
        theta = np.deg2rad(self.slope_deg)
        top_center = np.array([half_thickness * np.sin(theta), 0.0,
                               half_thickness * np.cos(theta)])
        shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[PLANE_HALF_EXTENT, PLANE_HALF_EXTENT,
                         half_thickness])
        orientation = p.getQuaternionFromEuler([0.0, theta, 0.0])
        return p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=shape,
            basePosition=(-top_center).tolist(),
            baseOrientation=orientation)

    def scan(self):
        """竖直向下的 N×N 射线，返回命中点坐标 (N², 3)。"""
        half = PLANE_HALF_EXTENT * 0.2
        axis = np.linspace(-half, half, self.grid_n)
        starts, ends = [], []
        for x in axis:
            for y in axis:
                starts.append([x, y, CAMERA_HEIGHT])
                ends.append([x, y, RAY_FLOOR])
        hits = p.rayTestBatch(starts, ends)
        points = []
        for start, end, hit in zip(starts, ends, hits):
            fraction = hit[2]
            if fraction >= 1.0:
                continue
            point = np.asarray(start) + fraction * (
                np.asarray(end) - np.asarray(start))
            points.append(point)
        return np.asarray(points, dtype=float)

    def estimate(self, points):
        """对点云做一次最小二乘与一次 RANSAC 拟合。"""
        normal_ls, d_ls = fit_plane_least_squares(points)
        normal_rs, d_rs, ratio = fit_plane_ransac(
            points, self.ransac_threshold, seed=self.seed)
        return {
            'slope_ls': np.rad2deg(slope_angle(normal_ls)),
            'slope_ransac': np.rad2deg(slope_angle(normal_rs)),
            'height_ls': plane_height(normal_ls, d_ls, 0.0, 0.0),
            'ransac_inlier_ratio': ratio,
            'inlier_ratio_ls': inlier_ratio(
                points, normal_ls, d_ls, self.ransac_threshold),
        }

    def run_once(self):
        points = self.scan()
        if points.shape[0] < 3:
            self.get_logger().error('射线未命中地面，请检查相机与地面位姿')
            return
        if self.noise_std > 0.0:
            rng = np.random.default_rng(self.seed)
            points[:, 2] += rng.normal(0.0, self.noise_std, points.shape[0])
        result = self.estimate(points)
        result.update({
            'slope_true': self.slope_deg,
            'noise_std': self.noise_std,
            'n_points': int(points.shape[0]),
        })
        message = Float64MultiArray()
        message.data = [result['slope_true'], result['slope_ls'],
                        result['slope_ransac'], result['height_ls']]
        self._publish(message)
        self._write(result)
        self.get_logger().info(
            '真值 {:.2f}°，最小二乘 {:.2f}°，RANSAC {:.2f}°，'
            '高程误差 {:+.4f} m，内点率 {:.2f}'.format(
                result['slope_true'], result['slope_ls'],
                result['slope_ransac'], result['height_ls'],
                result['ransac_inlier_ratio']))

    def _publish(self, message):
        """按 publish_rate 连续发布 publish_count 次。

        节点算完一次就退出，若只发布一次，后启动的 `ros2 topic echo`
        基本收不到；连续发布一段时间可以把结果留在总线上。
        """
        if self.publish_count <= 0:
            return
        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.1
        for _ in range(self.publish_count):
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=period)

    def _write(self, result):
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(
            self.out_dir,
            'scan_{:05.2f}deg_{:.1f}mm.csv'.format(
                self.slope_deg, self.noise_std * 1000.0))
        names = ('slope_true', 'slope_ls', 'slope_ransac', 'noise_std',
                 'n_points', 'height_ls', 'ransac_inlier_ratio',
                 'inlier_ratio_ls')
        with open(path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=names)
            writer.writeheader()
            writer.writerow({key: result[key] for key in names})
        self.get_logger().info('结果已写入 {}'.format(path))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SlopeScanNode()
    except RuntimeError as error:
        # PyBullet 后端不可用时给出可读错误，而不是抛 traceback
        rclpy.logging.get_logger('slope_sim').error(
            '节点初始化失败: {}'.format(error))
        rclpy.shutdown()
        return
    try:
        node.run_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
