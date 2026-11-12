# -*- coding: utf-8 -*-
"""巡线小车仿真的纯逻辑与 headless 回归测试。"""

import json
import os
import sys
import unittest

os.environ["MPLBACKEND"] = "Agg"

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import main  # noqa: E402


class PathTest(unittest.TestCase):
    def test_sample_path_closed_and_dense(self):
        path = main.sample_path(main.WAYPOINTS)
        self.assertGreater(len(path), 50)
        # 闭合路径：首尾点应接近
        self.assertAlmostEqual(path[0][0], path[-1][0], delta=1.0)
        self.assertAlmostEqual(path[0][1], path[-1][1], delta=1.0)

    def test_nearest_index(self):
        path = main.sample_path(main.WAYPOINTS)
        i, d = main.nearest_index(path, path[10][0], path[10][1])
        self.assertAlmostEqual(d, 0.0, places=6)
        self.assertEqual(i, 10)


class FollowerTest(unittest.TestCase):
    def _run(self, seed=42, steps=3000):
        path = main.sample_path(main.WAYPOINTS)
        sc = main.LineFollower(path, seed=seed)
        for _ in range(steps):
            sc.step()
            if sc.finished or sc.offline:
                break
        return sc

    def test_starts_on_line(self):
        path = main.sample_path(main.WAYPOINTS)
        sc = main.LineFollower(path)
        _, d = main.nearest_index(path, sc.x, sc.y)
        self.assertLess(d, main.LINE_HALF_WIDTH)

    def test_tracks_loop_without_offline(self):
        sc = self._run()
        m = sc.metrics()
        self.assertTrue(m["finished"])
        self.assertFalse(m["offline"])
        self.assertLess(m["max_abs_offset"], 3.0)

    def test_offline_flag_when_pushed_away(self):
        path = main.sample_path(main.WAYPOINTS)
        sc = main.LineFollower(path)
        sc.x -= 20.0
        sc.y += 20.0  # 推离到夹角外侧
        for _ in range(20):
            sc.step()
        self.assertTrue(sc.offline)

    def test_determinism_same_seed(self):
        m1 = self._run(seed=3).metrics()
        m2 = self._run(seed=3).metrics()
        self.assertEqual(json.dumps(m1, sort_keys=True),
                         json.dumps(m2, sort_keys=True))


if __name__ == "__main__":
    unittest.main()
