"""RRT 的单元测试。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import rrt as rrtmod


class TestGeometry(unittest.TestCase):
    def test_segment_rect_crossing(self):
        rect = (10.0, 10.0, 20.0, 20.0)
        self.assertTrue(rrtmod.segment_hits_rect((0.0, 15.0), (30.0, 15.0), rect))
        self.assertTrue(rrtmod.segment_hits_rect((0.0, 0.0), (15.0, 15.0), rect))
        self.assertFalse(rrtmod.segment_hits_rect((0.0, 0.0), (5.0, 5.0), rect))
        self.assertFalse(rrtmod.segment_hits_rect((0.0, 30.0), (30.0, 30.0), rect))

    def test_margin_expands_obstacle(self):
        rect = (10.0, 10.0, 20.0, 20.0)
        seg = ((0.0, 8.5), (30.0, 8.5))       # 距矩形 1.5 m
        self.assertFalse(rrtmod.segment_hits_rect(seg[0], seg[1], rect, 0.0))
        self.assertTrue(rrtmod.segment_hits_rect(seg[0], seg[1], rect, 1.5))

    def test_point_in_rect(self):
        rect = (0.0, 0.0, 10.0, 10.0)
        self.assertTrue(rrtmod.point_in_rect((5.0, 5.0), rect))
        self.assertFalse(rrtmod.point_in_rect((11.0, 5.0), rect))
        self.assertTrue(rrtmod.point_in_rect((11.0, 5.0), rect, margin=1.5))


class TestRRT(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner = rrtmod.RRT(rrtmod.START, rrtmod.GOAL, rrtmod.OBSTACLES, seed=3)
        cls.path = cls.planner.plan()

    def test_path_found(self):
        self.assertIsNotNone(self.path)
        self.assertGreaterEqual(len(self.path), 2)

    def test_endpoints(self):
        self.assertAlmostEqual(rrtmod.dist(self.path[0], rrtmod.START), 0.0, places=9)
        self.assertLess(rrtmod.dist(self.path[-1], rrtmod.GOAL), 1e-9)

    def test_path_collision_free(self):
        self.assertTrue(rrtmod.path_is_free(self.path, rrtmod.OBSTACLES))
        self.assertTrue(rrtmod.path_is_free(self.path, rrtmod.OBSTACLES, 1.0))

    def test_smoothing_shortens_and_stays_free(self):
        sm = rrtmod.smooth(self.path, rrtmod.OBSTACLES, iterations=300, seed=5)
        self.assertLessEqual(rrtmod.path_length(sm), rrtmod.path_length(self.path) + 1e-9)
        self.assertTrue(rrtmod.path_is_free(sm, rrtmod.OBSTACLES))
        self.assertGreater(len(self.path), len(sm))       # 顶点数减少

    def test_steer_respects_step(self):
        p = rrtmod.RRT((0.0, 0.0), (100.0, 100.0), [], step=5.0)
        q = p.steer((0.0, 0.0), (100.0, 0.0))
        self.assertAlmostEqual(rrtmod.dist((0.0, 0.0), q), 5.0, places=9)
        q2 = p.steer((0.0, 0.0), (1.0, 0.0))              # 比步长更近时直达
        self.assertAlmostEqual(q2[0], 1.0)

    def test_tree_edges_are_free(self):
        for a, b in self.planner.edges[:50]:
            self.assertTrue(rrtmod.segment_free(a, b, rrtmod.OBSTACLES))


class TestTracking(unittest.TestCase):
    """评审指出的回归：规划路径无碰撞，但跟踪时会切进障碍。

    旧实现把「累计弧长超过 lookahead 的下一个顶点」当前视目标；平滑后只剩
    约 10 个顶点，那个顶点可能在拐角之后，实测 13.2 s 撞进第一个障碍。
    """
    @classmethod
    def setUpClass(cls):
        import plan_control_node as pc
        cls.pc = pc
        planner = rrtmod.RRT(rrtmod.START, rrtmod.GOAL, rrtmod.OBSTACLES, seed=3)
        raw = planner.plan()
        cls.smoothed = rrtmod.smooth(raw, rrtmod.OBSTACLES, seed=5)
        cls.dense = rrtmod.densify(cls.smoothed, 0.5)

    def test_densify_keeps_endpoints_and_length(self):
        self.assertLessEqual(len(self.smoothed), 15)
        self.assertGreater(len(self.dense), 300)
        self.assertAlmostEqual(rrtmod.dist(self.dense[0], self.smoothed[0]), 0.0, places=9)
        self.assertAlmostEqual(rrtmod.dist(self.dense[-1], self.smoothed[-1]), 0.0, places=9)
        self.assertAlmostEqual(rrtmod.path_length(self.dense),
                               rrtmod.path_length(self.smoothed), places=6)
        for a, b in zip(self.dense[:-1], self.dense[1:]):
            self.assertLessEqual(rrtmod.dist(a, b), 0.5 + 1e-9)
        self.assertTrue(rrtmod.path_is_free(self.dense, rrtmod.OBSTACLES))

    def test_lookahead_point_is_at_arc_length_and_resolution_independent(self):
        line = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
        dense_line = rrtmod.densify(line, 0.5)
        for ld, expected in ((8.0, (8.0, 0.0)), (15.0, (10.0, 5.0))):
            p = rrtmod.lookahead_point(line, 0.0, 0.0, ld)
            q = rrtmod.lookahead_point(dense_line, 0.0, 0.0, ld)
            self.assertLess(rrtmod.dist(p, expected), 1e-9)
            self.assertLess(rrtmod.dist(p, q), 0.5)   # 插值版本与分辨率无关

    def test_tracking_trajectory_is_collision_free(self):
        """控制 20 Hz、运动学积分 50 Hz，全程不入任何障碍并最终抵达目标。"""
        dt, ctrl_dt = 0.02, 0.05
        x, y, yaw = 8.0, 8.0, 0.7854
        v = w = 0.0
        t = next_ctrl = 0.0
        reached = False
        for _ in range(int(150.0 / dt)):
            if t + 1e-12 >= next_ctrl:
                v, w, reached = self.pc.pure_pursuit_cmd(
                    self.dense, (x, y, yaw), lookahead=8.0, v_cruise=3.0)
                if reached:
                    break
                next_ctrl += ctrl_dt
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw = self.pc.wrap(yaw + w * dt)
            t += dt
            for i, rect in enumerate(rrtmod.OBSTACLES):
                self.assertFalse(
                    rrtmod.point_in_rect((x, y), rect),
                    "t=%.2f s 在 (%.3f, %.3f) 撞进障碍 #%d"
                    % (t, x, y, i))
        self.assertTrue(reached, "150 s 内未抵达目标")
        self.assertLess(rrtmod.dist((x, y), rrtmod.GOAL), 1.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
