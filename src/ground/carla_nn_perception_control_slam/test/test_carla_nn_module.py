#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单元测试：不依赖 CARLA 服务端，只测任务③的核心算法组件。

覆盖范围：
  - GridMap 栅格化 / log-odds 射线更新 / 占据判定 / 钳位
  - A* 全局规划（可达、绕障、封死返回空）
  - to_image 可视化输出形状

运行方式（Ubuntu 20.04 + ROS 2 Humble 虚拟机内）：
    cd src/ground/carla_nn_perception_control_slam
    python3 -m unittest discover -s test -v
"""
import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

from carla_nn_perception_control_slam import slam_nav


class TestGridMap(unittest.TestCase):

    def test_k_grid_resolution(self):
        gm = slam_nav.GridMap(res=0.5)
        self.assertEqual(gm._k(1.2, -0.3), (2, -1))
        self.assertEqual(gm._k(0.0, 0.49), (0, 0))
        self.assertEqual(gm._k(-0.51, 0.0), (-2, 0))

    def test_update_ray_marks_free_and_endpoint_occupied(self):
        gm = slam_nav.GridMap(res=0.5)
        gm.update_ray(0.0, 0.0, 10.0, 0.0)
        self.assertTrue(gm.is_occ(gm._k(10.0, 0.0)))
        mid = gm._k(5.0, 0.0)
        self.assertLess(gm.logodds.get(mid, 0.0), 0.0)
        self.assertTrue(gm.is_free(mid))

    def test_logodds_clamped_to_max(self):
        gm = slam_nav.GridMap(res=0.5)
        for _ in range(20):
            gm.update_ray(0.0, 0.0, 10.0, 0.0)
        self.assertLessEqual(gm.logodds[gm._k(10.0, 0.0)], gm.L_MAX)

    def test_short_ray_ignored(self):
        gm = slam_nav.GridMap(res=0.5)
        gm.update_ray(0.0, 0.0, 0.1, 0.0)  # 距离 < 0.3，应被忽略
        self.assertEqual(gm.logodds, {})

    def test_bounds_and_image_shape(self):
        gm = slam_nav.GridMap(res=0.5)
        gm.update_ray(0.0, 0.0, 10.0, 0.0)
        i0, j0, i1, j1 = gm.bounds()
        self.assertEqual((i0, i1), (0, 20))
        img, origin = gm.to_image()
        self.assertEqual(img.shape, (j1 - j0 + 1, i1 - i0 + 1))
        self.assertEqual(origin, (i0, j0))


class TestAStar(unittest.TestCase):

    def _wall_map(self):
        """在 x=5 处竖一面墙，仅留 (5, 3) 一个缺口。"""
        gm = slam_nav.GridMap(res=1.0)
        for j in range(-4, 5):
            if j == 3:
                continue
            gm.logodds[(5, j)] = 3.0
        return gm

    def test_astar_reaches_goal_through_gap(self):
        gm = self._wall_map()
        path = gm.astar((0, 0), (10, 0))
        self.assertTrue(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (10, 0))
        self.assertTrue(all(not gm.is_occ(k) for k in path))
        self.assertIn((5, 3), path)

    def test_astar_empty_on_blocked_map(self):
        gm = slam_nav.GridMap(res=1.0)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                gm.logodds[(10 + di, dj)] = 3.0  # 把 goal 完全围死
        self.assertEqual(gm.astar((0, 0), (10, 0)), [])


class TestShippedWeights(unittest.TestCase):
    """防回归：确保 PilotNet 权重路径指向真实存在的文件。

    覆盖 jydw121 在 PR #873 复核中指出的"路径改了一半"问题——
    MODEL 必须能解析到随包入库的 models/pilotnet.pt。
    路径解析已抽到 paths.py（只依赖 os + 可选 ament），CI 无 CARLA/cv2 也能跑。
    """

    def test_shipped_weights_path_exists(self):
        from carla_nn_perception_control_slam import paths
        self.assertTrue(os.path.exists(paths.MODEL), paths.MODEL)


if __name__ == "__main__":
    unittest.main()
