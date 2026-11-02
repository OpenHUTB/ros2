"""离线单元测试：LiDAR 聚类测距（parse_lidar_points / lidar_obstacle_distance）。"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from carla_aeb_braking.aeb_core import (  # noqa: E402
    lidar_obstacle_distance,
    parse_lidar_points,
)


def _cluster(x, z, n=40, spread=0.3):
    """在 (x, z) 处构造 n 个点的密集簇（模拟障碍车身面）。"""
    rng = np.random.default_rng(int(round(x * 10)))
    pts = np.zeros((n, 4), dtype=np.float32)
    pts[:, 0] = x + rng.uniform(-spread, spread, n)
    pts[:, 1] = rng.uniform(-1.0, 1.0, n)
    pts[:, 2] = z + rng.uniform(-0.15, 0.15, n)
    return pts


def _strays(xs, z=-0.8):
    """若干孤立近距杂波点（模拟自车车身/路侧回波）。"""
    pts = np.array([[x, 0.0, z, 0.0] for x in xs], dtype=np.float32)
    return pts


class TestLidarObstacleDistance(unittest.TestCase):
    def test_empty_returns_valid_max(self):
        self.assertEqual(lidar_obstacle_distance(np.zeros((0, 4))), 80.0)

    def test_far_obstacle_cluster(self):
        pts = np.vstack([_cluster(45.0, -0.9), _strays([4.9, 5.0, 5.1])])
        d = lidar_obstacle_distance(pts)
        self.assertAlmostEqual(d, 45.5, delta=1.0)

    def test_near_obstacle_cluster(self):
        pts = np.vstack([_cluster(7.0, -1.0, n=60), _strays([4.9, 5.0, 5.1])])
        d = lidar_obstacle_distance(pts)
        self.assertAlmostEqual(d, 7.5, delta=1.0)

    def test_ground_return_filtered(self):
        # 地面点（传感器系 z≈-2.0，低于高度带）不得被当作障碍
        ground = np.vstack([_cluster(45.0, -2.1), _strays([4.9, 5.0, 5.1])])
        self.assertEqual(lidar_obstacle_distance(ground), 80.0)

    def test_isolated_strays_ignored(self):
        # 只有孤立杂波点（每 bin 点数不足）→ 视为无障碍
        self.assertEqual(lidar_obstacle_distance(_strays([5.0, 9.5, 14.0])), 80.0)

    def test_x_min_excludes_ego_body(self):
        # 自车车身近距密集回波（x<3）被 x_min 滤除
        ego = _cluster(2.0, -0.8, n=50)
        self.assertEqual(lidar_obstacle_distance(ego), 80.0)

    def test_nearest_cluster_wins(self):
        # 两簇并存时取最近的一簇
        pts = np.vstack([_cluster(30.0, -0.9), _cluster(10.0, -1.0, n=50)])
        d = lidar_obstacle_distance(pts)
        self.assertAlmostEqual(d, 10.5, delta=1.0)

    def test_lateral_object_excluded(self):
        # 车道外（|y|>y_max）的密集簇不得被当作前向障碍
        pts = _cluster(20.0, -0.9, n=60)
        pts[:, 1] += 8.0
        self.assertEqual(lidar_obstacle_distance(pts), 80.0)


class TestParseLidarPoints(unittest.TestCase):
    def test_shape_and_values(self):
        raw = np.array([[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]],
                       dtype=np.float32).tobytes()
        pts = parse_lidar_points(raw)
        self.assertEqual(pts.shape, (2, 4))
        np.testing.assert_allclose(pts[1], [5.0, 6.0, 7.0, 8.0])


if __name__ == "__main__":
    unittest.main()
