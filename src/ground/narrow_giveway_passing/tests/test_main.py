# -*- coding: utf-8 -*-
"""窄道会车让行仿真的纯逻辑与 headless 回归测试。"""

import json
import os
import sys
import unittest

os.environ["MPLBACKEND"] = "Agg"

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import main  # noqa: E402


class NormalizeAngleTest(unittest.TestCase):
    def test_normalize(self):
        self.assertAlmostEqual(main.normalize_angle(0.1), 0.1)
        self.assertAlmostEqual(main.normalize_angle(3 * main.math.pi), main.math.pi)
        self.assertAlmostEqual(main.normalize_angle(-3 * main.math.pi), -main.math.pi)


class ScenarioTest(unittest.TestCase):
    def _run(self, seed=42, steps=3000):
        sc = main.Scenario(seed=seed)
        for _ in range(steps):
            sc.step()
            if sc.done:
                break
        return sc

    def test_yield_decision_picks_closer_car(self):
        sc = main.Scenario()
        self.assertEqual(sc.yield_car.role, "A")
        self.assertEqual(sc.pass_car.role, "B")

    def test_passes_without_collision(self):
        sc = self._run()
        m = sc.metrics()
        self.assertTrue(m["passed"])
        self.assertEqual(m["collisions"], 0)
        self.assertGreater(m["min_headway"], 0.0)

    def test_yield_car_waits_then_exits(self):
        sc = main.Scenario()
        # 让行车必须经历 waiting 阶段，且等待时间为正
        saw_waiting = False
        for _ in range(3000):
            sc.step()
            if sc.yield_car.state == "waiting":
                saw_waiting = True
            if sc.done:
                break
        self.assertTrue(saw_waiting)
        self.assertGreater(sc.yield_car.wait_time, 0.0)

    def test_determinism_same_seed(self):
        m1 = self._run(seed=7).metrics()
        m2 = self._run(seed=7).metrics()
        self.assertEqual(json.dumps(m1, sort_keys=True),
                         json.dumps(m2, sort_keys=True))


if __name__ == "__main__":
    unittest.main()
