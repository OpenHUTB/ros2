# -*- coding: utf-8 -*-
"""人工势场法（APF）路径规划离线单元测试。

不依赖 pygame，纯算法层验证。
"""

import math
import unittest

from apf_path_planning.apf import (
    APFParams,
    Obstacle,
    attractive_potential,
    attractive_gradient,
    repulsive_potential,
    repulsive_gradient,
    total_force,
    step,
    plan,
    distance,
    obstacle_distance,
    potential_at,
)


class TestParams(unittest.TestCase):
    """参数校验。"""

    def test_valid_params(self):
        APFParams().validate()

    def test_negative_k_att_raises(self):
        p = APFParams(k_att=-1.0)
        with self.assertRaises(ValueError):
            p.validate()

    def test_zero_step_size_raises(self):
        p = APFParams(step_size=0.0)
        with self.assertRaises(ValueError):
            p.validate()

    def test_plan_validates_params(self):
        with self.assertRaises(ValueError):
            plan((0, 0), (1, 1), [], APFParams(k_rep=-1.0))


class TestDistance(unittest.TestCase):
    """距离函数。"""

    def test_zero(self):
        self.assertAlmostEqual(distance((3, 4), (3, 4)), 0.0)

    def test_pythagorean(self):
        self.assertAlmostEqual(distance((0, 0), (3, 4)), 5.0)

    def test_negative(self):
        self.assertAlmostEqual(distance((-1, -1), (2, 3)), 5.0)


class TestObstacleDistance(unittest.TestCase):
    def test_outside(self):
        self.assertAlmostEqual(obstacle_distance((5, 0), Obstacle(0, 0, 2.0)), 3.0)

    def test_on_boundary(self):
        self.assertAlmostEqual(obstacle_distance((2, 0), Obstacle(0, 0, 2.0)), 0.0)

    def test_inside_negative(self):
        self.assertAlmostEqual(obstacle_distance((1, 0), Obstacle(0, 0, 2.0)), -1.0)


class TestAttractivePotential(unittest.TestCase):
    """引力势函数。"""

    def test_zero_at_goal(self):
        self.assertAlmostEqual(attractive_potential((1, 1), (1, 1), 1.0, 2.0), 0.0)

    def test_near_field_quadratic(self):
        # d=1 ≤ d_thresh=2: U = 0.5*1*1^2 = 0.5
        self.assertAlmostEqual(attractive_potential((0, 0), (1, 0), 1.0, 2.0), 0.5)

    def test_far_field_cone(self):
        # d=3 > d_thresh=2: U = 1*2*3 - 0.5*1*4 = 6 - 2 = 4
        self.assertAlmostEqual(attractive_potential((0, 0), (3, 0), 1.0, 2.0), 4.0)

    def test_continuity_at_threshold(self):
        k, d0 = 1.0, 2.0
        u_near = attractive_potential((0, 0), (1.999, 0), k, d0)
        u_far = attractive_potential((0, 0), (2.001, 0), k, d0)
        self.assertAlmostEqual(u_near, u_far, places=1)


class TestAttractiveGradient(unittest.TestCase):
    def test_zero_at_goal(self):
        g = attractive_gradient((1, 1), (1, 1), 1.0, 2.0)
        self.assertAlmostEqual(g[0], 0.0)
        self.assertAlmostEqual(g[1], 0.0)

    def test_points_away_from_goal(self):
        # pos=(0,0) goal=(1,0): grad 应指向 -x（势能增加方向 = 远离 goal）
        g = attractive_gradient((0, 0), (1, 0), 1.0, 2.0)
        self.assertLess(g[0], 0.0)

    def test_far_field_constant_magnitude(self):
        g = attractive_gradient((0, 0), (5, 0), 1.0, 2.0)
        self.assertAlmostEqual(math.hypot(*g), 1.0 * 2.0, places=6)


class TestRepulsivePotential(unittest.TestCase):
    def test_outside_threshold(self):
        obs = [Obstacle(5.0, 0.0, 1.0)]
        # d=4 > d_thresh=1.5：U=0
        self.assertAlmostEqual(repulsive_potential((0.0, 0.0), obs, 100.0, 1.5), 0.0)

    def test_inside_threshold_positive(self):
        obs = [Obstacle(5.0, 0.0, 1.0)]
        u = repulsive_potential((4.0, 0.0), obs, 100.0, 1.5)
        self.assertGreater(u, 0.0)

    def test_inside_obstacle_inf(self):
        obs = [Obstacle(5.0, 0.0, 1.0)]
        self.assertEqual(repulsive_potential((5.5, 0.0), obs, 100.0, 1.5),
                         float("inf"))

    def test_no_obstacles_zero(self):
        self.assertAlmostEqual(repulsive_potential((1, 2), [], 100.0, 1.5), 0.0)


class TestRepulsiveGradient(unittest.TestCase):
    def test_points_toward_obstacle(self):
        """斥力梯度 ∇U_rep 指向势能增加方向（即障碍物方向），
        合力取负后才远离障碍物。pos 在障碍物左侧时 gx 应为正。"""
        obs = [Obstacle(5.0, 0.0, 1.0)]
        g = repulsive_gradient((3.5, 0.0), obs, 100.0, 1.5)
        self.assertGreater(g[0], 0.0)

    def test_outside_threshold_zero(self):
        obs = [Obstacle(5.0, 0.0, 1.0)]
        g = repulsive_gradient((0.0, 0.0), obs, 100.0, 1.5)
        self.assertAlmostEqual(g[0], 0.0)
        self.assertAlmostEqual(g[1], 0.0)


class TestTotalForce(unittest.TestCase):
    def test_no_obstacles_equals_attraction(self):
        params = APFParams()
        pos = (1.0, 0.0)
        goal = (3.0, 0.0)
        f = total_force(pos, goal, [], params)
        ag = attractive_gradient(pos, goal, params.k_att, params.d_thresh_att)
        self.assertAlmostEqual(f[0], -ag[0])
        self.assertAlmostEqual(f[1], -ag[1])

    def test_force_points_to_goal_without_obstacle(self):
        params = APFParams()
        f = total_force((0, 0), (5, 0), [], params)
        self.assertGreater(f[0], 0.0)


class TestStep(unittest.TestCase):
    def test_moves_toward_goal(self):
        params = APFParams(step_size=0.1, goal_tol=0.1)
        pos = (0.0, 0.0)
        goal = (5.0, 0.0)
        new_pos, _ = step(pos, goal, [], params)
        self.assertGreater(new_pos[0], pos[0])

    def test_step_size_bound(self):
        params = APFParams(step_size=0.1, goal_tol=0.1)
        new_pos, _ = step((0, 0), (5, 0), [], params)
        self.assertLessEqual(distance(new_pos, (0, 0)), 0.1 + 1e-9)


class TestPlanNoObstacles(unittest.TestCase):
    """无障碍物场景：直线到达。"""

    def test_reached(self):
        params = APFParams(step_size=0.1, goal_tol=0.15, max_iter=500)
        path, status = plan((0.0, 0.0), (5.0, 0.0), [], params)
        self.assertEqual(status, "reached")
        self.assertLessEqual(distance(path[-1], (5.0, 0.0)), 0.15 + 1e-6)

    def test_path_starts_at_start(self):
        params = APFParams(step_size=0.1, goal_tol=0.15, max_iter=500)
        path, _ = plan((0.0, 0.0), (5.0, 0.0), [], params)
        self.assertAlmostEqual(path[0][0], 0.0)
        self.assertAlmostEqual(path[0][1], 0.0)


class TestPlanWithObstacles(unittest.TestCase):
    """侧方障碍场景：应到达且避开障碍。"""

    def setUp(self):
        self.params = APFParams(k_att=1.0, k_rep=100.0,
                                d_thresh_att=2.0, d_thresh_rep=1.5,
                                step_size=0.1, goal_tol=0.2,
                                max_iter=1000)

    def test_sideways_obstacle_avoided(self):
        # 障碍在路径侧上方，路径应绕过且不撞
        obs = [Obstacle(2.5, 0.5, 0.4)]
        path, status = plan((0.0, 0.0), (5.0, 0.0), obs, self.params)
        self.assertEqual(status, "reached")
        min_d = min(obstacle_distance(p, obs[0]) for p in path)
        self.assertGreater(min_d, 0.0, "路径穿入障碍物")

    def test_path_avoids_all_obstacles(self):
        obs = [Obstacle(2.0, 0.5, 0.3), Obstacle(3.5, -0.5, 0.3)]
        path, status = plan((0.0, 0.0), (5.0, 0.0), obs, self.params)
        self.assertEqual(status, "reached")
        for p in path:
            for o in obs:
                self.assertGreater(obstacle_distance(p, o), 0.0,
                                   "路径穿入障碍物")


class TestPotentialAt(unittest.TestCase):
    """总势能（引力 + 斥力）。"""

    def test_goal_zero_without_obstacles(self):
        params = APFParams()
        self.assertAlmostEqual(potential_at((1, 1), (1, 1), [], params), 0.0)

    def test_positive_away_from_goal(self):
        params = APFParams()
        self.assertGreater(potential_at((0, 0), (3, 0), [], params), 0.0)


if __name__ == "__main__":
    unittest.main()