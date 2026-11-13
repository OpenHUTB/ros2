# -*- coding: utf-8 -*-
"""RRT 路径规划离线单元测试。

不依赖 pygame，覆盖障碍碰撞检测、最近邻、步长受限扩展、
路径回溯、自由空间可达性与起点被封死的不可达判定。

运行：在 src/ground/rrt_path_planning/ 目录下执行
    python -m pytest test/test_rrt.py -v
"""
import os
import sys
import unittest

# 让 test 能 import 上级包
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from rrt_path_planning.map import ObstacleMap
from rrt_path_planning.rrt import RRT, Node
from rrt_path_planning import config as C


class TestObstacleMap(unittest.TestCase):

    def setUp(self):
        self.m = ObstacleMap([(100, 100, 80, 60)], 400, 300)

    def test_point_inside_obstacle(self):
        self.assertTrue(self.m.point_in_obstacle(120, 130))
        self.assertTrue(self.m.point_in_obstacle(100, 100))   # 左上角含边界
        self.assertTrue(self.m.point_in_obstacle(180, 160))   # 右下角含边界

    def test_point_outside_obstacle(self):
        self.assertFalse(self.m.point_in_obstacle(50, 50))
        self.assertFalse(self.m.point_in_obstacle(200, 100))
        self.assertFalse(self.m.point_in_obstacle(140, 170))  # 紧贴下边外侧

    def test_segment_clear_of_obstacles(self):
        # 完全在障碍外的线段
        self.assertFalse(self.m.segment_collides(10, 10, 90, 250))

    def test_segment_through_obstacle(self):
        # 穿过矩形的线段
        self.assertTrue(self.m.segment_collides(50, 130, 250, 130))
        # 对角穿过
        self.assertTrue(self.m.segment_collides(80, 80, 220, 200))

    def test_segment_endpoints_inside(self):
        # 端点在障碍内也算碰撞
        self.assertTrue(self.m.segment_collides(120, 130, 130, 140))


class TestRRTNearestAndSteer(unittest.TestCase):

    def setUp(self):
        self.m = ObstacleMap([], 400, 300)
        self.r = RRT(self.m, (10, 10), (390, 290),
                     step_size=20, goal_radius=15,
                     goal_sample_rate=0.0, seed=1)
        # 手工构造一棵小树
        self.r.tree = [Node(0, 0), Node(10, 0), Node(0, 10), Node(100, 100)]

    def test_nearest_node(self):
        n = self.r.nearest(self.r.tree, 12, 1)
        self.assertAlmostEqual(n.x, 10)
        self.assertAlmostEqual(n.y, 0)
        n2 = self.r.nearest(self.r.tree, 95, 105)
        self.assertAlmostEqual(n2.x, 100)
        self.assertAlmostEqual(n2.y, 100)

    def test_steer_step_limit(self):
        from_node = Node(0, 0)
        new = self.r.steer(from_node, 100, 0)   # 距离 100，步长 20
        self.assertAlmostEqual(new.x, 20)
        self.assertAlmostEqual(new.y, 0)
        self.assertEqual(new.parent, from_node)

    def test_steer_short_distance(self):
        from_node = Node(0, 0)
        new = self.r.steer(from_node, 5, 5)     # 距离约 7.07 < step_size 20
        self.assertAlmostEqual(new.x, 5)
        self.assertAlmostEqual(new.y, 5)

    def test_steer_diagonal_direction(self):
        from_node = Node(0, 0)
        new = self.r.steer(from_node, 30, 40)   # 距离 50，步长 20 -> ratio 0.4
        self.assertAlmostEqual(new.x, 12)
        self.assertAlmostEqual(new.y, 16)


class TestRRTPlan(unittest.TestCase):

    def test_plan_finds_path_in_free_space(self):
        m = ObstacleMap([], 400, 300)
        r = RRT(m, (10, 10), (390, 290),
                step_size=20, goal_radius=15,
                goal_sample_rate=0.10, seed=42)
        path = r.plan(max_iter=2000)
        self.assertGreater(len(path), 1, "自由空间应能找到路径")
        self.assertAlmostEqual(path[0].x, 10)
        self.assertAlmostEqual(path[0].y, 10)
        # 终点应在目标半径内
        self.assertLessEqual(path[-1].dist_to(390, 290), 15)

    def test_plan_blocked_start_returns_empty(self):
        # 用一圈障碍把起点完全围死（含采样空间边界）
        walls = [
            (0, 0, 50, 5),     # 上墙
            (0, 45, 50, 5),    # 下墙
            (0, 0, 5, 50),     # 左墙
            (45, 0, 5, 50),    # 右墙
        ]
        m = ObstacleMap(walls, 400, 300)
        r = RRT(m, (25, 25), (390, 290),
                step_size=10, goal_radius=15,
                goal_sample_rate=0.05, seed=7)
        path = r.plan(max_iter=500)
        self.assertEqual(path, [], "起点被围死应返回空路径")

    def test_plan_with_obstacles_finds_path(self):
        # 默认障碍布局（config 中的布局）
        m = ObstacleMap(C.OBSTACLES, C.WIDTH, C.HEIGHT)
        r = RRT(m, C.START, C.GOAL,
                step_size=C.STEP_SIZE, goal_radius=C.GOAL_RADIUS,
                goal_sample_rate=C.GOAL_SAMPLE_RATE, seed=C.SEED)
        path = r.plan(max_iter=C.MAX_ITER)
        self.assertGreater(len(path), 1, "默认布局应能找到路径")
        # 路径上任意相邻节点连线都不应穿过障碍
        for i in range(len(path) - 1):
            self.assertFalse(
                m.segment_collides(path[i].x, path[i].y,
                                   path[i + 1].x, path[i + 1].y),
                f"路径第 {i} 段穿过障碍")

    def test_retrace_chain_order(self):
        a = Node(0, 0)
        b = Node(1, 1, parent=a)
        c = Node(2, 2, parent=b)
        d = Node(3, 3, parent=c)
        m = ObstacleMap([], 100, 100)
        r = RRT(m, (0, 0), (3, 3), 10, 5, 0.0, seed=1)
        path = r.retrace(d)
        self.assertEqual(len(path), 4)
        self.assertEqual(path[0], a)
        self.assertEqual(path[-1], d)

    def test_path_length_zero_and_nonzero(self):
        m = ObstacleMap([], 100, 100)
        r = RRT(m, (0, 0), (10, 0), 10, 5, 0.0, seed=1)
        self.assertEqual(r.path_length([]), 0.0)
        self.assertEqual(r.path_length([Node(0, 0)]), 0.0)
        path = [Node(0, 0), Node(3, 0), Node(3, 4)]
        self.assertAlmostEqual(r.path_length(path), 7.0)

    def test_seed_reproducibility(self):
        """同一种子两次规划结果应完全一致。"""
        m = ObstacleMap(C.OBSTACLES, C.WIDTH, C.HEIGHT)
        r1 = RRT(m, C.START, C.GOAL, C.STEP_SIZE, C.GOAL_RADIUS,
                 C.GOAL_SAMPLE_RATE, seed=42)
        r2 = RRT(m, C.START, C.GOAL, C.STEP_SIZE, C.GOAL_RADIUS,
                 C.GOAL_SAMPLE_RATE, seed=42)
        p1 = r1.plan(C.MAX_ITER)
        p2 = r2.plan(C.MAX_ITER)
        self.assertEqual(len(p1), len(p2))
        for a, b in zip(p1, p2):
            self.assertAlmostEqual(a.x, b.x)
            self.assertAlmostEqual(a.y, b.y)


if __name__ == "__main__":
    unittest.main(verbosity=2)
