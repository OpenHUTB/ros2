"""BSD 决策核心离线单元测试（不依赖 ROS 与 CARLA）。

运行：python3 main.py test
"""

import unittest

import numpy as np

from carla_bsd_blind_spot.bsd_core import (
    NORMAL, WARN, BSDController, BSDParams, blind_zone_distance,
    bsd_decide, estimate_closing, parse_lidar_points,
)


def make_car_body(cx, cy, n=30, z=-1.0):
    """在 (cx, cy) 附近生成一片车身高度带的连续表面点（模拟障碍车）。"""
    rng = np.random.default_rng(7)
    xs = cx + rng.uniform(-0.8, 0.8, n)
    ys = cy + rng.uniform(-0.8, 0.8, n)
    zs = np.full(n, z) + rng.uniform(-0.2, 0.2, n)
    return np.stack([xs, ys, zs, np.full(n, 0.5)], axis=1).astype(np.float32)


class TestBlindZoneDistance(unittest.TestCase):

    def test_empty_returns_valid_max(self):
        self.assertEqual(
            blind_zone_distance(np.zeros((0, 4), dtype=np.float32)),
            40.0)

    def test_left_zone_detection(self):
        # 左盲区窗口内（y=3.0, x=-4.0 → r=5.0）的车身应被检测到
        pts = make_car_body(-4.0, 3.0)
        d = blind_zone_distance(pts, side="left")
        self.assertLess(d, 8.0)
        self.assertGreater(d, 3.0)

    def test_right_zone_symmetric(self):
        pts = make_car_body(-4.0, -3.0)
        d = blind_zone_distance(pts, side="right")
        self.assertLess(d, 8.0)

    def test_lateral_outside_window_missed(self):
        # y=1.0 在近界 1.8 之内（非盲区车道）→ 不应报目标
        pts = make_car_body(-4.0, 1.0)
        self.assertEqual(blind_zone_distance(pts, side="left"), 40.0)

    def test_longitudinal_outside_window_missed(self):
        # x=6.0 在 x_max=2.0 之前（正前方，属于前向雷达职责）→ 不报
        pts = make_car_body(6.0, 3.0)
        self.assertEqual(blind_zone_distance(pts, side="left"), 40.0)

    def test_ground_filtered_by_height_band(self):
        # 地面点世界 z=0 → 传感器 z≈-2.0，低于 z_min=-1.7 → 不报
        pts = make_car_body(-4.0, 3.0, z=-2.0)
        self.assertEqual(blind_zone_distance(pts, side="left"), 40.0)

    def test_roadside_marker_filtered(self):
        # 2 m 路侧标志：传感器 z≈0，高于 z_max=-0.4 → 不报
        pts = make_car_body(-4.0, 3.0, z=0.0)
        self.assertEqual(blind_zone_distance(pts, side="left"), 40.0)

    def test_isolated_clutter_missed(self):
        # 少于 min_points 的孤立杂波不构成目标
        pts = make_car_body(-4.0, 3.0, n=3)
        self.assertEqual(blind_zone_distance(pts, side="left"), 40.0)

    def test_nearest_cluster_wins(self):
        # 5 m 与 12 m 两个目标，取最近
        pts = np.vstack([make_car_body(-4.0, 3.0),      # r≈5
                         make_car_body(-11.0, 4.5)])    # r≈11.9
        d = blind_zone_distance(pts, side="left")
        self.assertLess(d, 7.0)

    def test_side_validation(self):
        with self.assertRaises(ValueError):
            blind_zone_distance(make_car_body(0, 3.0), side="front")

    def test_custom_window(self):
        # 收窄远界后，远目标不再可见
        pts = make_car_body(-11.0, 4.5)
        self.assertEqual(
            blind_zone_distance(pts, side="left", x_max=2.0, x_min=-8.0), 40.0)


class TestEstimateClosing(unittest.TestCase):

    def test_positive_when_approaching(self):
        self.assertAlmostEqual(estimate_closing(10.0, 9.0, 0.05), 20.0)

    def test_negative_when_receding(self):
        self.assertAlmostEqual(estimate_closing(9.0, 10.0, 0.05), -20.0)

    def test_nonpositive_dt(self):
        self.assertEqual(estimate_closing(10.0, 9.0, 0.0), 0.0)


class TestBSDDecide(unittest.TestCase):

    def setUp(self):
        self.p = BSDParams()

    def test_normal_to_warn(self):
        state, _ = bsd_decide(NORMAL, 10.0, 0.0, self.p)
        self.assertEqual(state, WARN)

    def test_normal_stays_when_far(self):
        state, _ = bsd_decide(NORMAL, 13.0, 0.0, self.p)
        self.assertEqual(state, NORMAL)

    def test_warn_recovery_uses_exit_threshold(self):
        # 13 m 在 [warn_enter=12, warn_exit=15] 滞回带内 → warn 保持
        state, _ = bsd_decide(WARN, 13.0, 0.0, self.p)
        self.assertEqual(state, WARN)
        state, _ = bsd_decide(WARN, 16.0, 0.0, self.p)
        self.assertEqual(state, NORMAL)

    def test_approach_flag(self):
        _, a = bsd_decide(NORMAL, 20.0, 1.5, self.p)   # 20<25 且正在接近
        self.assertTrue(a)
        _, a = bsd_decide(NORMAL, 20.0, -1.0, self.p)  # 在远离
        self.assertFalse(a)
        _, a = bsd_decide(NORMAL, 30.0, 3.0, self.p)   # 太远
        self.assertFalse(a)

    def test_illegal_state_raises(self):
        with self.assertRaises(ValueError):
            bsd_decide("braking", 10.0, 0.0, self.p)


class TestBSDController(unittest.TestCase):

    def test_warn_event_emitted_once(self):
        c = BSDController()
        events = []
        for _ in range(5):
            r = c.tick(10.0, 40.0)
            events.append(r["left"]["event"])
        self.assertEqual(events[0], "left_warn")
        self.assertTrue(all(e == "" for e in events[1:]))

    def test_sides_independent(self):
        c = BSDController()
        r = c.tick(10.0, 40.0)
        self.assertEqual(r["left"]["state"], WARN)
        self.assertEqual(r["right"]["state"], NORMAL)

    def test_recovery_after_target_leaves(self):
        c = BSDController()
        c.tick(10.0, 40.0)
        r = c.tick(18.0, 40.0)   # >warn_exit → 恢复
        self.assertEqual(r["left"]["state"], NORMAL)

    def test_approach_early_warning(self):
        c = BSDController()
        r = c.tick(22.0, 40.0)
        self.assertFalse(r["left"]["approach"])   # 尚无接近速度历史
        r = c.tick(20.0, 40.0, dt=0.05)           # 2m/0.05s=40m/s 接近
        self.assertTrue(r["left"]["approach"])
        self.assertEqual(r["left"]["state"], NORMAL)  # 但状态仍 normal

    def test_reset(self):
        c = BSDController()
        c.tick(10.0, 10.0)
        c.reset()
        r = c.tick(10.0, 10.0)
        self.assertEqual(r["left"]["event"], "left_warn")  # 复位后重新报警


class TestParseLidarPoints(unittest.TestCase):

    def test_roundtrip(self):
        pts = np.array([[1, 2, 3, 4]], dtype=np.float32)
        out = parse_lidar_points(pts.tobytes())
        self.assertEqual(out.shape, (1, 4))
        np.testing.assert_array_equal(out, pts)

    def test_length_must_be_multiple_of_16(self):
        with self.assertRaises(ValueError):
            parse_lidar_points(b"\x00" * 15)


if __name__ == "__main__":
    unittest.main()
