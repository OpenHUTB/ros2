# -*- coding: utf-8 -*-
"""圆形轨迹纯数学部分的测试（不需要 ROS）。"""

from __future__ import annotations

import csv
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros_topic_demo.circle_path import (  # noqa: E402
    normalize_angle,
    pose_at,
    sample_poses,
    write_csv,
)


class AngleTest(unittest.TestCase):
    def test_normalize(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(2 * math.pi + 0.1), 0.1)
        self.assertAlmostEqual(normalize_angle(-2 * math.pi - 0.1), -0.1)


class PoseTest(unittest.TestCase):
    def test_start_on_positive_x(self):
        x, y, yaw = pose_at(0.0, radius=1.0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_quarter_turn(self):
        x, y, yaw = pose_at(math.pi / 2, radius=2.0)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(abs(yaw), math.pi)

    def test_radius_is_preserved(self):
        for phase in (0.0, 0.3, 1.2, 2.5, 4.0):
            x, y, _ = pose_at(phase, radius=1.5)
            self.assertAlmostEqual(math.hypot(x, y), 1.5)

    def test_yaw_is_tangent(self):
        # 逆时针运动的切向 = phase + pi/2
        for phase in (0.0, 0.7, 1.9):
            _, _, yaw = pose_at(phase)
            self.assertAlmostEqual(
                math.sin(yaw - (phase + math.pi / 2)), 0.0, places=9)

    def test_invalid_radius(self):
        with self.assertRaises(ValueError):
            pose_at(0.0, radius=0.0)


class SampleTest(unittest.TestCase):
    def test_one_period_sample_count(self):
        rows = sample_poses(period=10.0, rate=10.0, radius=1.0)
        self.assertEqual(len(rows), 100)
        self.assertAlmostEqual(rows[0]["t"], 0.0)
        self.assertAlmostEqual(rows[-1]["t"], 9.9)
        self.assertAlmostEqual(rows[0]["phase"], 0.0)

    def test_full_circle(self):
        rows = sample_poses(period=4.0, rate=8.0, radius=1.0)
        self.assertEqual(len(rows), 32)
        # 一圈内绕原点一周
        total = 0.0
        for r in rows:
            self.assertAlmostEqual(math.hypot(r["x"], r["y"]), 1.0)
        for a, b in zip(rows, rows[1:]):
            total += b["phase"] - a["phase"]
        self.assertGreater(total, 2 * math.pi * 0.9)

    def test_invalid_parameters(self):
        for kwargs in ({"period": 0}, {"rate": 0}, {"radius": 0},
                       {"period": 0.01, "rate": 0.1}):
            with self.assertRaises(ValueError):
                sample_poses(**kwargs)


class CsvTest(unittest.TestCase):
    def test_write_csv(self):
        rows = sample_poses(period=2.0, rate=2.0)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "circle.csv")
            write_csv(path, rows)
            with open(path, encoding="utf-8", newline="") as f:
                data = list(csv.reader(f))
        self.assertEqual(data[0], ["t", "phase", "x", "y", "yaw"])
        self.assertEqual(len(data), len(rows) + 1)


if __name__ == "__main__":
    unittest.main()
