"""apf（人工势场）的离线单元测试。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import apf


class TestGeometry(unittest.TestCase):
    def test_dist_to_rect_external(self):
        self.assertAlmostEqual(apf.dist_to_rect((0, 0), (10, 10, 5, 5)), math.hypot(10, 10))

    def test_dist_to_rect_edge_and_inside(self):
        self.assertAlmostEqual(apf.dist_to_rect((12, 0), (10, 10, 5, 5)), 10.0)
        self.assertAlmostEqual(apf.dist_to_rect((12, 12), (10, 10, 5, 5)), 0.0)

    def test_attractive_gradient(self):
        # ∇U_att = zeta * (q - goal) —— 指向背离 goal（力才是朝向 goal）
        gx, gy = apf.attractive_gradient((3, 1), (5, 7), zeta=2.0)
        self.assertAlmostEqual(gx, -4.0)
        self.assertAlmostEqual(gy, -12.0)

    def test_repulsive_matches_numeric(self):
        # 解析梯度 vs 中心差分数值梯度（对照含引力的总梯度）
        obs = [(10.0, 10.0, 5.0, 5.0)]
        goal = (100.0, 100.0)
        kw = dict(eta=100.0, d0=4.0)
        h = 1e-6
        for q in [(6.0, 12.0), (8.0, 9.0), (15.0, 16.0), (12.0, 17.5), (7.3, 14.1)]:
            ax, ay = apf.total_gradient(q, goal, obs, **kw)
            nx = (apf.potential((q[0] + h, q[1]), goal, obs, **kw)
                  - apf.potential((q[0] - h, q[1]), goal, obs, **kw)) / (2 * h)
            ny = (apf.potential((q[0], q[1] + h), goal, obs, **kw)
                  - apf.potential((q[0], q[1] - h), goal, obs, **kw)) / (2 * h)
            self.assertAlmostEqual(ax, nx, delta=1e-3, msg=str(q))
            self.assertAlmostEqual(ay, ny, delta=1e-3, msg=str(q))

    def test_repulsive_zero_beyond_d0(self):
        obs = [(0.0, 0.0, 4.0, 4.0)]
        self.assertEqual(apf.repulsive_gradient((10.0, 0.0), obs, 100.0, 4.0), (0.0, 0.0))
        # 距表面恰好 4.0（d == d0，边界处为 0）
        self.assertEqual(apf.repulsive_gradient((8.0, 2.0), obs, 100.0, 4.0), (0.0, 0.0))

    def test_total_gradient_attracts_to_goal(self):
        # 远离一切障碍时合成梯度 ≈ 引力
        gx, gy = apf.total_gradient((0.0, 0.0), (10.0, 0.0), [(20.0, 20.0, 2.0, 2.0)])
        self.assertAlmostEqual(gx, -10.0, delta=1e-6)
        self.assertAlmostEqual(gy, 0.0, delta=1e-6)


class TestNavigate(unittest.TestCase):
    def setUp(self):
        self.res = apf.navigate(apf.START, apf.GOAL, apf.OBSTACLES)

    def test_standard_reaches_goal(self):
        self.assertTrue(self.res["success"])
        end = self.res["path"][-1]
        self.assertLess(math.hypot(end[0] - apf.GOAL[0], end[1] - apf.GOAL[1]), 1.5)

    def test_standard_no_collision(self):
        self.assertGreater(self.res["min_clear"], 0.0)

    def test_standard_length_reasonable(self):
        direct = math.hypot(apf.GOAL[0] - apf.START[0], apf.GOAL[1] - apf.START[1])
        self.assertGreater(self.res["length"], direct)
        self.assertLess(self.res["length"], direct * 3.0)

    def test_step_size_bounded(self):
        p = self.res["path"]
        for i in range(len(p) - 1):
            d = math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1])
            self.assertLessEqual(d, 0.5 + 1e-9)

    def test_trap_stuck_without_escape(self):
        r = apf.navigate(apf.TRAP_START, apf.TRAP_GOAL, apf.TRAP_OBSTACLES,
                         escape=False, max_iter=3000)
        self.assertFalse(r["success"])
        self.assertIsNotNone(r["stuck"])
        # 卡点应在墙前（goal 在墙后 y=41，墙占 y∈[30,36]）
        self.assertLess(r["stuck"][1], 30.0)

    def test_trap_escape_succeeds(self):
        for seed in (3, 42):
            r = apf.navigate(apf.TRAP_START, apf.TRAP_GOAL, apf.TRAP_OBSTACLES,
                             escape=True, seed=seed, max_iter=6000)
            self.assertTrue(r["success"], msg="seed=%d" % seed)
            self.assertGreater(r["min_clear"], 0.0)

    def test_trap_escape_stays_in_map(self):
        # 评审 #1400：切向滑行必须在越过墙端、滑出斥力影响半径 d0 后
        # 立刻退出回到梯度模式——旧版固定滑 60 步（30 m），越过墙端
        # 20 m 还冲出 50 m 地图边界。锁死：1 次逃逸、路径不出地图。
        for seed in (7, 3, 42, 99):
            r = apf.navigate(apf.TRAP_START, apf.TRAP_GOAL, apf.TRAP_OBSTACLES,
                             escape=True, seed=seed, max_iter=6000)
            self.assertTrue(r["success"], msg="seed=%d" % seed)
            self.assertEqual(r["escapes"], 1, msg="seed=%d" % seed)
            for x, y in r["path"]:
                self.assertGreaterEqual(x, 0.0, msg="seed=%d" % seed)
                self.assertLessEqual(x, 50.0, msg="seed=%d" % seed)
                self.assertGreaterEqual(y, 0.0, msg="seed=%d" % seed)
                self.assertLessEqual(y, 50.0, msg="seed=%d" % seed)

    def test_escape_retries_bounded(self):
        # 逃逸次数有上限，不会无限循环
        r = apf.navigate(apf.TRAP_START, apf.TRAP_GOAL, apf.TRAP_OBSTACLES,
                         escape=False, max_iter=3000)
        self.assertLessEqual(r["escapes"], 13)

    def test_step_command_unit_direction(self):
        d, u = apf.step_command((0.0, 0.0), (10.0, 0.0), [(5.0, 5.0, 1.0, 1.0)])
        self.assertAlmostEqual(math.hypot(d[0], d[1]), 1.0)
        # 无障碍干扰时指向 goal
        self.assertAlmostEqual(d[0], 1.0, delta=1e-6)
        self.assertAlmostEqual(d[1], 0.0, delta=1e-6)


class TestTracking(unittest.TestCase):
    """plan_control_node.track_cmd 的闭环回放（无 ROS，直接积分运动学）。"""

    def _replay(self, path, goal, v=2.0, rate=20.0, t_max=120.0):
        import plan_control_node as pcn
        x, y, yaw = path[0][0], path[0][1], 0.0
        idx = 0
        dt = 1.0 / rate
        t = 0.0
        while t < t_max:
            v_, w, idx, done = pcn.track_cmd(path, (x, y, yaw), idx, speed=v,
                                             goal=goal)
            if done:
                return True, t
            yaw += w * dt
            x += v_ * math.cos(yaw) * dt
            y += v_ * math.sin(yaw) * dt
            t += dt
        return False, t

    def test_replay_reaches_goal(self):
        r = apf.navigate(apf.START, apf.GOAL, apf.OBSTACLES)
        ok, t = self._replay(r["path"], apf.GOAL)
        self.assertTrue(ok)
        self.assertLess(t, 90.0)

    def test_replay_collision_free(self):
        r = apf.navigate(apf.START, apf.GOAL, apf.OBSTACLES)
        ok, _ = self._replay(r["path"], apf.GOAL)
        self.assertTrue(ok)
        # 复用 navigate 的轨迹本身无碰撞，这里校验回放始终离参考轨迹不远
        # —— 有限转速 (w_max=2) 不会让车甩出安全裕度。
        # 简单判据：回放轨迹所有点到最近障碍的距离 > 0.3 m。
        import plan_control_node as pcn
        x, y, yaw = r["path"][0][0], r["path"][0][1], 0.0
        idx, dt, t = 0, 0.05, 0.0
        while t < 90.0:
            v_, w, idx, done = pcn.track_cmd(r["path"], (x, y, yaw), idx,
                                             speed=2.0, goal=apf.GOAL)
            if done:
                break
            yaw += w * dt
            x += v_ * math.cos(yaw) * dt
            y += v_ * math.sin(yaw) * dt
            t += dt
            self.assertGreater(apf.nearest_obstacle_distance((x, y), apf.OBSTACLES),
                               0.3)

    def test_track_cmd_goal_reached_stops(self):
        import plan_control_node as pcn
        path = [(0.0, 0.0), (1.0, 0.0)]
        v, w, idx, done = pcn.track_cmd(path, (1.2, 0.0, 0.0), 0,
                                        speed=2.0, goal=(1.0, 0.0))
        self.assertTrue(done)
        self.assertEqual((v, w), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
