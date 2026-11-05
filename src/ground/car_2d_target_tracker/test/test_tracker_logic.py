#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2D 小车目标追踪：单元测试。

    python3 test/test_tracker_logic.py

覆盖：
  * 几何与碰撞（角度归一化、圆面间隙、**扫掠碰撞**——防"一步跨过障碍"穿模）
  * 单轮车模型（前进方向、撞上障碍不穿透、被挡计数）
  * 控制器（空旷场地能逼近静止目标；两种模式在"被挡"场景下的差别）
  * 指标（首次到达时刻、路径长度、卡死占比）
  * 仓库约定（main.* / launch / package.xml / README 齐全，launch 参数真被使用）

本模块**不依赖 pygame / 图形界面**，因此这些用例在无头机器上全部可跑。
"""
from __future__ import annotations

import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import math as _m  # noqa: E402

from car_2d_target_tracker import geometry as geo          # noqa: E402
from car_2d_target_tracker.car import Car                  # noqa: E402
from car_2d_target_tracker.config import Scenario, TrackerConfig  # noqa: E402
from car_2d_target_tracker.controller import Tracker       # noqa: E402
from car_2d_target_tracker.metrics import (first_reach_time, path_length,  # noqa: E402
                                           stuck_fraction, summarize)
from car_2d_target_tracker.runner import compare, default_scenarios, run_scenario  # noqa: E402
from car_2d_target_tracker.target import Target            # noqa: E402


# ---------------------------------------------------------------------------
# 几何
# ---------------------------------------------------------------------------
class TestGeometry(unittest.TestCase):
    def test_normalize_angle(self):
        self.assertAlmostEqual(geo.normalize_angle(3 * math.pi), math.pi)
        self.assertAlmostEqual(geo.normalize_angle(-3 * math.pi), math.pi)
        self.assertAlmostEqual(geo.normalize_angle(0.5), 0.5)

    def test_point_circle_gap_sign(self):
        self.assertAlmostEqual(geo.point_circle_gap(0, 0, 0, 0, 10), -10.0)
        self.assertAlmostEqual(geo.point_circle_gap(20, 0, 0, 0, 10), 10.0)

    def test_sweep_detects_tunneling(self):
        """回归：原实现只检查终点，一步跨过障碍就会"穿模"。

        这里构造"起点与终点都在障碍外、但连线穿过障碍"的情形，扫掠必须判定为撞。
        """
        obstacles = ((100.0, 100.0, 20.0),)
        self.assertTrue(geo.collides(100, 100, 5, obstacles))          # 终点在障碍里
        self.assertFalse(geo.collides(10, 100, 5, obstacles))          # 起点在外
        self.assertFalse(geo.collides(200, 100, 5, obstacles))         # 终点也在外
        self.assertTrue(geo.sweep_hits(10, 100, 200, 100, 5, obstacles))   # 但连线穿过了

    def test_sweep_free_path(self):
        obstacles = ((100.0, 300.0, 20.0),)
        self.assertFalse(geo.sweep_hits(10, 100, 200, 100, 5, obstacles))

    def test_nearest_obstacle(self):
        obstacles = ((100, 0, 10), (300, 0, 10))
        near = geo.nearest_obstacle(0, 0, obstacles)
        self.assertAlmostEqual(near[0], 100.0)
        self.assertIsNone(geo.nearest_obstacle(0, 0, ()))


# ---------------------------------------------------------------------------
# 车辆模型
# ---------------------------------------------------------------------------
class TestCar(unittest.TestCase):
    def test_moves_along_heading(self):
        car = Car(0, 0, 0.0, radius=5)
        car.step(100.0, 0.0, 0.1, obstacles=())
        self.assertAlmostEqual(car.x, 10.0, places=6)
        self.assertAlmostEqual(car.y, 0.0, places=6)

    def test_cannot_penetrate_obstacle(self):
        """撞上障碍时车必须停在表面之外，不能穿进去。"""
        obstacles = ((50.0, 0.0, 20.0),)
        car = Car(0, 0, 0.0, radius=5)
        for _ in range(200):
            car.step(180.0, 0.0, 1 / 60.0, obstacles=obstacles)
        gap = geo.point_circle_gap(car.x, car.y, 50.0, 0.0, 20.0)
        self.assertGreaterEqual(gap, 5.0 - 1e-6, f"车穿进了障碍：gap={gap:.3f}")
        self.assertGreater(car.blocked_steps, 0)

    def test_high_speed_step_does_not_tunnel(self):
        """即使一步跨度远大于障碍直径，也不能穿过去。"""
        obstacles = ((100.0, 0.0, 10.0),)
        car = Car(0, 0, 0.0, radius=5)
        car.step(5000.0, 0.0, 0.1, obstacles=obstacles)      # 一步 500 px
        self.assertLess(car.x, 100.0)


# ---------------------------------------------------------------------------
# 目标与控制器
# ---------------------------------------------------------------------------
class TestTargetAndController(unittest.TestCase):
    def test_target_seed_is_reproducible(self):
        a = Target(0, 0, seed=7)
        b = Target(0, 0, seed=7)
        self.assertAlmostEqual(a.vx, b.vx)
        self.assertAlmostEqual(a.vy, b.vy)

    def test_target_stays_inside_field(self):
        tgt = Target(890, 590, width=900, height=600, margin=25, seed=1)
        for _ in range(2000):
            tgt.step(1 / 60.0)
            self.assertTrue(25 - 1e-6 <= tgt.x <= 900 - 25 + 1e-6)
            self.assertTrue(25 - 1e-6 <= tgt.y <= 600 - 25 + 1e-6)

    def test_approach_static_target(self):
        """空旷场地上，baseline 应当能逼近静止目标。"""
        cfg = TrackerConfig()
        car = Car(100, 300, 0.0, radius=cfg.car_radius)
        tgt = Target(500, 300, motion="static", seed=0)
        ctrl = Tracker(cfg, mode="baseline")
        for _ in range(600):
            v, omega, arrived = ctrl.compute(car, tgt)
            if arrived:
                break
            car.step(v, omega, cfg.dt, obstacles=())
        self.assertLess(car.distance_to(tgt.x, tgt.y), cfg.arrive_radius)


# ---------------------------------------------------------------------------
# 场景对照（集成）
# ---------------------------------------------------------------------------
class TestScenarios(unittest.TestCase):
    def test_blocked_scenario_separates_the_two_modes(self):
        """核心回归：障碍正挡在目标方向上时，baseline 卡死、escape 必须能到达。"""
        cfg = TrackerConfig()
        sc = [s for s in default_scenarios(cfg) if s.name == "blocked"][0]
        _s1, m_base = run_scenario(cfg, sc, mode="baseline")
        _s2, m_esc = run_scenario(cfg, sc, mode="escape")
        self.assertFalse(m_base["reached"], f"baseline 竟然到达了：{m_base}")
        self.assertTrue(m_esc["reached"], f"escape 未到达：{m_esc}")
        self.assertLess(m_esc["arrive_time"], sc.sim_time)

    def test_chase_scenario_escape_catches_moving_target(self):
        """追移动目标：原速度律（120 px 减速半径）追不上，改进版必须追得上。"""
        cfg = TrackerConfig()
        sc = [s for s in default_scenarios(cfg) if s.name == "chase"][0]
        _s1, m_base = run_scenario(cfg, sc, mode="baseline")
        _s2, m_esc = run_scenario(cfg, sc, mode="escape")
        self.assertFalse(m_base["reached"])
        self.assertTrue(m_esc["reached"])

    def test_gate_scenario_both_pass(self):
        """非阻挡场景下两版都该通过（说明改进没有把原有能力改坏）。"""
        cfg = TrackerConfig()
        sc = [s for s in default_scenarios(cfg) if s.name == "gate"][0]
        _s1, m_base = run_scenario(cfg, sc, mode="baseline")
        _s2, m_esc = run_scenario(cfg, sc, mode="escape")
        self.assertTrue(m_base["reached"])
        self.assertTrue(m_esc["reached"])

    def test_compare_returns_both_modes(self):
        cfg = TrackerConfig(sim_time=2.0)
        res = compare(cfg, [Scenario("t", (100, 100, 0.0), (300, 100),
                                     target_motion="static", obstacles=(), sim_time=2.0)],
                      verbose=False)
        self.assertIn("baseline", res["t"])
        self.assertIn("escape", res["t"])


# ---------------------------------------------------------------------------
# 指标
# ---------------------------------------------------------------------------
class TestMetrics(unittest.TestCase):
    def test_first_reach_time(self):
        self.assertAlmostEqual(first_reach_time([0, 1, 2], [50, 10, 5], 18), 1.0)
        self.assertTrue(math.isnan(first_reach_time([0, 1], [50, 40], 18)))

    def test_path_length(self):
        self.assertAlmostEqual(path_length([0, 3], [0, 4]), 5.0)

    def test_stuck_fraction(self):
        self.assertAlmostEqual(stuck_fraction([0, 1, 2, 3], [0, 0, 50, 0], float("nan"), 5), 0.75)
        self.assertAlmostEqual(stuck_fraction([0, 1, 2], [50, 50, 50], float("nan"), 5), 0.0)

    def test_summarize(self):
        m = summarize([0, 1, 2], [0, 1, 2], [0, 0, 0], [10, 10, 0], [100, 10, 5],
                      [5, 5, 5], arrive_radius=18, speed_eps=5)
        self.assertTrue(m["reached"])
        self.assertAlmostEqual(m["arrive_time"], 1.0)
        self.assertAlmostEqual(m["path_length"], 2.0)


# ---------------------------------------------------------------------------
# 仓库约定
# ---------------------------------------------------------------------------
class TestRepoConventions(unittest.TestCase):
    def test_required_files(self):
        names = ("main.py", "package.xml", "setup.py", "requirements.txt",
                 "README.md", "launch/main.launch.py")
        for rel in names:
            self.assertTrue(os.path.isfile(os.path.join(_PKG, rel)), f"缺少 {rel}")
        self.assertTrue(os.path.isfile(os.path.join(_PKG, "main.bat")))
        self.assertTrue(os.path.isfile(os.path.join(_PKG, "main.sh")))

    def test_launch_args_declared_and_used(self):
        import re
        text = open(os.path.join(_PKG, "launch", "main.launch.py"), encoding="utf-8").read()
        declared = set(re.findall(r"DeclareLaunchArgument\(\s*\n?\s*['\"]([^'\"]+)", text))
        used = set(re.findall(r"LaunchConfiguration\(\s*['\"]([^'\"]+)", text))
        self.assertTrue(declared)
        self.assertTrue(declared <= used, f"声明了但没使用：{sorted(declared - used)}")

    def test_headless_does_not_import_pygame(self):
        """默认路径不能 import pygame：无 pygame 的机器也要能跑完整评测。"""
        src = open(os.path.join(_PKG, "car_2d_target_tracker", "cli.py"),
                   encoding="utf-8").read()
        self.assertNotIn("import pygame", src)
        runner = open(os.path.join(_PKG, "car_2d_target_tracker", "runner.py"),
                      encoding="utf-8").read()
        self.assertNotIn("import pygame", runner)


if __name__ == "__main__":
    unittest.main(verbosity=2)
