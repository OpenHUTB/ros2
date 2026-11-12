# -*- coding: utf-8 -*-
"""2D 小车目标追踪的纯逻辑与 headless 回归测试。"""

from __future__ import annotations

import json
import math
import os
import random
import sys
import tempfile
import unittest
from unittest import mock

# 必须在 import main / pygame 之前设置，避免 CI/无显示器机器尝试打开真实窗口
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import main  # noqa: E402


class AngleTest(unittest.TestCase):
    def test_normalize(self):
        self.assertAlmostEqual(main.normalize_angle(0.1), 0.1)
        self.assertAlmostEqual(main.normalize_angle(3 * math.pi), math.pi)
        self.assertAlmostEqual(main.normalize_angle(-3 * math.pi), -math.pi)
        self.assertAlmostEqual(main.normalize_angle(2 * math.pi + 0.2), 0.2)


class CarTest(unittest.TestCase):
    def test_auto_step_moves_toward_target(self):
        car = main.Car(100.0, 100.0, 0.0)
        car.auto_step(1.0 / 60.0, 300.0, 100.0)
        self.assertGreater(car.x, 100.0)
        self.assertAlmostEqual(car.y, 100.0, places=6)

    def test_auto_step_stops_inside_arrive_radius(self):
        car = main.Car(100.0, 100.0, 0.0)
        arrived = car.auto_step(1.0 / 60.0, 105.0, 100.0)
        self.assertTrue(arrived)
        self.assertEqual(car.v, 0.0)

    def test_auto_step_counts_collision(self):
        # 障碍物 (300,200,r=35)，车半径 10，边界在 x=255
        car = main.Car(254.5, 200.0, 0.0)
        car.auto_step(1.0 / 60.0, 400.0, 200.0)
        self.assertEqual(car.collisions, 1)
        self.assertLess(car.x, 300.0)
        # 撞墙后即使不前进，也必须更新 theta；否则会永远贴在边界
        self.assertNotEqual(car.theta, 0.0)

    def test_collision_counts_contact_not_frames(self):
        car = main.Car(254.5, 200.0, 0.0)
        for _ in range(10):
            car.auto_step(1.0 / 60.0, 400.0, 200.0)
        self.assertEqual(car.collisions, 1)

    def test_escape_gets_around_obstacle_blocking_the_target(self):
        """回归：障碍正压在"车→目标"连线上时，必须绕过去，而不是贴着墙原地打转。

        旧行为：撞上后 ``v = 0``、只按最大角速度原地转。实测该场景 30 s 里
        95.6% 的时间速度低于 5 px/s，末距始终 440 px，永远到不了目标。
        """
        obstacle = [(460.0, 300.0, 90.0)]      # 正压在 (120,300) → (800,300) 中间
        with mock.patch.object(main, "OBSTACLES", obstacle):
            car = main.Car(120.0, 300.0, 0.0)
            dt = 1.0 / 60.0
            arrived_at = None
            slow_frames = 0
            frames = 0
            for i in range(int(30.0 / dt)):
                arrived = car.auto_step(dt, 800.0, 300.0)
                frames += 1
                if abs(car.v) < 5.0:
                    slow_frames += 1
                if arrived:
                    arrived_at = (i + 1) * dt
                    break
            self.assertIsNotNone(arrived_at, "绕不过障碍：30 s 内没到达目标")
            self.assertLess(arrived_at, 15.0, "绕行太慢：%.2f s" % arrived_at)
            # 也不能靠"一直不动"混过去：卡死时间占比要明显低于旧实现的 95.6%
            self.assertLess(slow_frames / max(1, frames), 0.40)

    def test_escape_follows_obstacle_surface_not_into_it(self):
        """绕行时车不能穿进障碍里：与最近障碍的间隙始终 >= 0（容差 1e-6）。"""
        obstacle = [(460.0, 300.0, 90.0)]
        with mock.patch.object(main, "OBSTACLES", obstacle):
            car = main.Car(120.0, 300.0, 0.0)
            dt = 1.0 / 60.0
            worst = float("inf")
            for _ in range(int(20.0 / dt)):
                car.auto_step(dt, 800.0, 300.0)
                ox, oy, r = obstacle[0]
                gap = math.hypot(car.x - ox, car.y - oy) - r - main.CAR_RADIUS
                worst = min(worst, gap)
            self.assertGreaterEqual(worst, -1e-6, "车穿进了障碍：gap=%.4f px" % worst)

    def test_manual_collision_counts_once(self):
        car = main.Car(254.5, 200.0, 0.0)
        keys = {k: False for k in (main.pygame.K_w, main.pygame.K_a,
                                   main.pygame.K_s, main.pygame.K_d,
                                   main.pygame.K_UP, main.pygame.K_DOWN,
                                   main.pygame.K_LEFT, main.pygame.K_RIGHT)}
        keys[main.pygame.K_w] = True
        for _ in range(10):
            car.manual_step(1.0 / 60.0, keys)
        self.assertEqual(car.collisions, 1)

    def test_manual_rotates_while_blocked(self):
        car = main.Car(254.5, 200.0, 0.0)
        keys = {k: False for k in (main.pygame.K_w, main.pygame.K_a,
                                   main.pygame.K_s, main.pygame.K_d,
                                   main.pygame.K_UP, main.pygame.K_DOWN,
                                   main.pygame.K_LEFT, main.pygame.K_RIGHT)}
        keys[main.pygame.K_w] = True
        keys[main.pygame.K_a] = True
        car.manual_step(1.0 / 60.0, keys)
        self.assertGreater(abs(car.theta), 0.0)
        self.assertGreaterEqual(car.collisions, 1)


class TargetTest(unittest.TestCase):
    def test_rng_is_deterministic(self):
        a = main.MovingTarget(700, 150, random.Random(1))
        b = main.MovingTarget(700, 150, random.Random(1))
        self.assertAlmostEqual(a.vx, b.vx)
        self.assertAlmostEqual(a.vy, b.vy)

    def test_bounces_at_right_edge(self):
        target = main.MovingTarget(main.WIDTH - 26, 100, random.Random(0))
        target.vx = 60.0
        target.vy = 0.0
        target.step(0.1)
        self.assertLess(target.vx, 0.0)
        self.assertLessEqual(target.x, main.WIDTH - 25)


class HeadlessRunTest(unittest.TestCase):
    def _run(self, tmp, seed=42, duration=20.0):
        return main.run(headless=True, seed=seed, duration=duration,
                        outdir=tmp, quiet=True)

    def test_metrics_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            m = self._run(tmp, duration=2.0)
            self.assertEqual(m["steps"], round(2.0 / main.FIXED_DT))
            self.assertTrue(os.path.exists(os.path.join(tmp, "metrics.json")))
            with open(os.path.join(tmp, "metrics.json"), encoding="utf-8") as f:
                disk = json.load(f)
            self.assertEqual(disk["steps"], m["steps"])
            self.assertGreaterEqual(disk["mean_distance"], 0.0)

    def test_catches_moving_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            m = self._run(tmp, seed=42, duration=20.0)
            self.assertTrue(m["arrived_once"])
            self.assertIsNotNone(m["time_to_target"])

    def test_deterministic_with_same_seed(self):
        with tempfile.TemporaryDirectory() as a, \
                tempfile.TemporaryDirectory() as b:
            ma = self._run(a, seed=7, duration=5.0)
            mb = self._run(b, seed=7, duration=5.0)
            for key in ("steps", "arrived_once", "time_to_target",
                        "mean_distance", "final_distance", "path_length",
                        "collisions"):
                self.assertEqual(ma[key], mb[key], key)

    def test_mouse_click_is_ignored_in_manual_mode(self):
        """README promises mouse target placement only in automatic mode."""

        def run_with_events(tmp, events):
            calls = 0

            def event_get():
                nonlocal calls
                calls += 1
                return list(events) if calls == 1 else []

            with mock.patch.object(main.pygame.event, "get",
                                   side_effect=event_get):
                return main.run(headless=True, seed=42, duration=2.0,
                                outdir=tmp, quiet=True)

        switch_to_manual = main.pygame.event.Event(
            main.pygame.KEYDOWN, key=main.pygame.K_SPACE)
        click = main.pygame.event.Event(
            main.pygame.MOUSEBUTTONDOWN, button=1, pos=(100, 500))

        with tempfile.TemporaryDirectory() as baseline_dir, \
                tempfile.TemporaryDirectory() as clicked_dir:
            baseline = run_with_events(baseline_dir, [switch_to_manual])
            clicked = run_with_events(clicked_dir, [switch_to_manual, click])

        self.assertEqual(baseline["mode"], "manual")
        for key in ("mean_distance", "final_distance", "path_length",
                    "collisions"):
            self.assertEqual(baseline[key], clicked[key], key)


class DemoTest(unittest.TestCase):
    def test_demo_saves_three_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            m = main.run(demo=True, headless=True, seed=42,
                         outdir=tmp, quiet=True)
            self.assertEqual(m["steps"], 601)
            for name in ("shot_demo_1_auto_chasing.png",
                         "shot_demo_2_avoiding.png",
                         "shot_demo_3_manual.png"):
                path = os.path.join(tmp, name)
                self.assertTrue(os.path.exists(path), name)
                self.assertGreater(os.path.getsize(path), 1000)


class CliTest(unittest.TestCase):
    def test_cli_headless_returns_zero(self):
        with tempfile.TemporaryDirectory() as tmp:
            code = main.main(["--headless", "--duration", "1",
                              "--seed", "3", "--outdir", tmp, "--quiet"])
            self.assertEqual(code, 0)
            self.assertTrue(os.path.exists(os.path.join(tmp, "metrics.json")))


if __name__ == "__main__":
    unittest.main()
