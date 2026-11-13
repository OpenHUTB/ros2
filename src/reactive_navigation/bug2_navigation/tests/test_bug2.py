"""bug2（反应式导航状态机）的离线单元测试。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import bug2


class TestRectBoundary(unittest.TestCase):
    def test_boundary_roundtrip(self):
        rect = (10.0, 10.0, 6.0, 4.0)
        per = bug2.rect_perimeter(rect)
        for s in (0.0, 1.7, 6.0, 9.9, 12.0, 15.5, per - 0.1, per, per + 2.3):
            p = bug2.boundary_point(rect, s)
            s2, p2 = bug2.project_to_boundary(p, rect)
            self.assertLessEqual(abs(s2 - (s % per)), 1e-6, msg="s=%s" % s)
            self.assertLessEqual(math.hypot(p[0] - p2[0], p[1] - p2[1]), 1e-6)

    def test_boundary_corners(self):
        rect = (10.0, 10.0, 6.0, 4.0)
        self.assertAlmostEqual(bug2.boundary_point(rect, 0.0)[0], 10.0)
        self.assertAlmostEqual(bug2.boundary_point(rect, 6.0)[0], 16.0)   # 右上角
        self.assertAlmostEqual(bug2.boundary_point(rect, 10.0)[1], 14.0)  # 右下角
        self.assertAlmostEqual(bug2.boundary_point(rect, 16.0)[0], 10.0)  # 左下角

    def test_project_inside_pushes_to_nearest_edge(self):
        rect = (0.0, 0.0, 10.0, 10.0)
        s, p = bug2.project_to_boundary((5.0, 2.0), rect)
        self.assertAlmostEqual(p[1], 0.0)      # 最近的边是上边 y=0
        self.assertAlmostEqual(s, 5.0)


class TestNavigate(unittest.TestCase):
    def test_no_obstacle_straight(self):
        r = bug2.navigate((2.0, 2.0), (48.0, 48.0), [])
        self.assertTrue(r["success"])
        self.assertLess(r["length"], math.hypot(46, 46) * 1.05)
        self.assertEqual(r["events"], [])

    def test_standard_scenario_success(self):
        r = bug2.navigate(bug2.START, bug2.GOAL, bug2.OBSTACLES)
        self.assertTrue(r["success"])
        self.assertGreater(len(r["events"]), 0)       # 至少一次 hit/leave

    def test_standard_no_collision(self):
        r = bug2.navigate(bug2.START, bug2.GOAL, bug2.OBSTACLES)
        for p in r["path"]:
            for o in bug2.OBSTACLES:
                self.assertGreaterEqual(bug2.rect_distance(p, o), 1.2 - 1e-9)

    def test_hit_leave_on_mline_and_closer(self):
        r = bug2.navigate(bug2.START, bug2.GOAL, bug2.OBSTACLES)
        d_goal = lambda p: math.hypot(p[0] - bug2.GOAL[0], p[1] - bug2.GOAL[1])
        pairs = [i for i, (e, _) in enumerate(r["events"]) if e in ("hit", "leave")]
        # hit 与 leave 成对出现
        hits = [e for e, _ in r["events"] if e == "hit"]
        leaves = [e for e, _ in r["events"] if e == "leave"]
        self.assertEqual(len(hits), len(leaves))
        for e, p in r["events"]:
            if e in ("hit", "leave"):
                self.assertLessEqual(
                    bug2.dist_to_segment(p, bug2.START, bug2.GOAL), 0.7)
        # 每个 leave 都比它前面的 hit 离目标更近
        last_hit = None
        for e, p in r["events"]:
            if e == "hit":
                last_hit = p
            elif e == "leave":
                self.assertLess(d_goal(p), d_goal(last_hit))

    def test_goal_inside_obstacle_rejected(self):
        r = bug2.navigate((2.0, 2.0), (18.0, 26.0), bug2.OBSTACLES)
        self.assertFalse(r["success"])
        self.assertEqual(len(r["path"]), 1)

    def test_walled_goal_unreachable(self):
        r = bug2.navigate(bug2.WALL_START, bug2.WALL_GOAL, bug2.WALLS)
        self.assertFalse(r["success"])
        kinds = [e for e, _ in r["events"]]
        self.assertIn("hit", kinds)
        self.assertIn("blocked", kinds)

    def test_trap_wall_reached_where_apf_stuck(self):
        # 与势场法相同的厚墙陷阱：Bug2 保证到达（完备性）
        r = bug2.navigate((30.0, 5.0), (30.0, 41.0), [(20.0, 30.0, 20.0, 6.0)])
        self.assertTrue(r["success"])
        end = r["path"][-1]
        self.assertLess(math.hypot(end[0] - 30.0, end[1] - 41.0), 1.5)

    def test_path_step_continuity(self):
        r = bug2.navigate(bug2.START, bug2.GOAL, bug2.OBSTACLES)
        p = r["path"]
        for i in range(len(p) - 1):
            d = math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1])
            self.assertLessEqual(d, 0.5 + 1e-9)

    def test_adjacent_obstacles_report_blocked_known_limitation(self):
        # 评审 #1400 复核（阻塞 2）：两块相距 1 m 的相邻障碍并不封死
        # （目标可从障碍外侧绕行），但"撞到相邻墙体 ⇒ 封死腔"的保守
        # 启发式判据会误报不可达。已作为已知限制写进文档（docs 第 1 节
        # 适用范围），本测试把行为锁死：它是确定性的。若未来实现联合
        # 边界绕行使本测试失败，请同步更新文档的适用范围说明。
        obs = [(20.0, 20.0, 4.0, 4.0), (25.0, 20.0, 4.0, 4.0)]
        r = bug2.navigate((5.0, 5.0), (45.0, 42.0), obs, r_clear=1.2)
        self.assertFalse(r["success"])
        kinds = [e for e, _ in r["events"]]
        self.assertIn("hit", kinds)
        self.assertIn("blocked", kinds)
        # 失败也计入 detours（一次沿墙以失败收尾，撞相邻墙终止）
        self.assertEqual(r["detours"], 1)
        self.assertAlmostEqual(r["length"], 25.43, delta=0.2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
