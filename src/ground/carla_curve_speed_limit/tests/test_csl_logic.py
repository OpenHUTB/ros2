"""CSL 决策核心离线单元测试（不依赖 ROS 与 CARLA）。

运行：python3 main.py test
"""

import math
import unittest

import numpy as np

from carla_curve_speed_limit.curve_speed_core import (
    CRUISE, SLOWDOWN, CSLController, CSLParams,
    curve_speed_limit, csl_decide, ramp_speed,
    signed_curvature, target_speed_along, waypoint_curvatures,
)


def arc_points(radius, n=40, total_angle=math.pi / 2, cx=0.0):
    """生成圆弧路点（从原点出发沿 +x 进入半径 radius 的圆弧）。"""
    cy = radius
    pts = [(0.0, 0.0)]
    for i in range(1, n):
        ang = total_angle * i / n
        pts.append((cx + radius * math.sin(ang),
                    cy - radius * math.cos(ang)))
    return pts


def straight_then_curve(straight_len, radius, n_straight=10, n_arc=30):
    """直道接圆弧：先沿 +x 直行，再进入半径 radius 的左转圆弧。"""
    pts = [(straight_len * i / n_straight, 0.0) for i in range(n_straight + 1)]
    x0, y0 = pts[-1]
    for i in range(1, n_arc):
        ang = (math.pi / 2) * i / n_arc
        pts.append((x0 + radius * math.sin(ang), y0 + radius * (1 - math.cos(ang))))
    return pts


class TestSignedCurvature(unittest.TestCase):

    def test_collinear_is_zero(self):
        k = signed_curvature((0, 0), (1, 0), (2, 0))
        self.assertAlmostEqual(k, 0.0)

    def test_known_radius(self):
        # 半径 10 的圆弧上取三点，|kappa| ≈ 1/10
        pts = arc_points(10.0, n=40)
        k = signed_curvature(pts[5], pts[10], pts[15])
        self.assertAlmostEqual(abs(k), 0.1, delta=0.005)

    def test_sign_left_positive(self):
        pts = arc_points(10.0, n=40)   # 左转（y 增大方向）
        k = signed_curvature(pts[5], pts[10], pts[15])
        self.assertGreater(k, 0)

    def test_identical_points_zero(self):
        self.assertEqual(signed_curvature((1, 1), (1, 1), (1, 1)), 0.0)


class TestCurveSpeedLimit(unittest.TestCase):

    def test_straight_returns_valid_max(self):
        self.assertEqual(curve_speed_limit(0.0), 30.0)
        self.assertEqual(curve_speed_limit(1e-9), 30.0)

    def test_v_equals_sqrt_aR(self):
        # R=10, a=2.5 → v = sqrt(25) = 5.0
        v = curve_speed_limit(0.1, a_lat_max=2.5)
        self.assertAlmostEqual(v, 5.0, delta=1e-6)

    def test_sharper_curve_slower(self):
        self.assertLess(curve_speed_limit(0.2), curve_speed_limit(0.1))

    def test_valid_max_cap(self):
        # 极缓弯（R=1000）理论速度远超上限，应被 valid_max 封顶
        self.assertEqual(curve_speed_limit(0.001, valid_max=30.0), 30.0)


class TestWaypointCurvatures(unittest.TestCase):

    def test_straight_all_zero(self):
        pts = [(i, 0.0) for i in range(10)]
        np.testing.assert_array_less(np.abs(waypoint_curvatures(pts)), 1e-9)

    def test_arc_uniform(self):
        k = waypoint_curvatures(arc_points(10.0, n=40))
        mid = k[10:30]
        self.assertTrue(np.all(np.abs(mid - 0.1) < 0.01))

    def test_short_input_zeros(self):
        np.testing.assert_array_equal(waypoint_curvatures([(0, 0), (1, 1)]),
                                      [0.0, 0.0])


class TestTargetSpeedAlong(unittest.TestCase):

    def test_pure_straight(self):
        pts = [(i, 0.0) for i in range(20)]
        self.assertEqual(target_speed_along(pts), 30.0)

    def test_pure_arc(self):
        pts = arc_points(10.0, n=40)
        v = target_speed_along(pts, a_lat_max=2.5)
        self.assertAlmostEqual(v, 5.0, delta=0.1)

    def test_straight_then_curve_limited_by_curve(self):
        pts = straight_then_curve(straight_len=20.0, radius=10.0)
        v = target_speed_along(pts, a_lat_max=2.5)
        # 最严限速由弯道决定，而不是直道
        self.assertAlmostEqual(v, 5.0, delta=0.15)

    def test_empty_input(self):
        self.assertEqual(target_speed_along([]), 30.0)


class TestCSLDecide(unittest.TestCase):

    def setUp(self):
        self.p = CSLParams()

    def test_cruise_to_slowdown(self):
        # 目标 5.0 比巡航 8.0 低 3.0 > 0.5 → slowdown
        state, ev = csl_decide(CRUISE, 5.0, self.p)
        self.assertEqual(state, SLOWDOWN)
        self.assertEqual(ev, "slowdown_warn")

    def test_cruise_stays_when_close(self):
        state, ev = csl_decide(CRUISE, 7.8, self.p)  # 差 0.2 < 0.5
        self.assertEqual(state, CRUISE)
        self.assertEqual(ev, "")

    def test_hysteresis_no_flap(self):
        # slowdown 中差距 0.3（在 0.2~0.5 滞回带内）→ 保持 slowdown
        state, ev = csl_decide(SLOWDOWN, 7.7, self.p)
        self.assertEqual(state, SLOWDOWN)
        self.assertEqual(ev, "")
        # 差距 < 0.2 → 恢复
        state, ev = csl_decide(SLOWDOWN, 7.9, self.p)
        self.assertEqual(state, CRUISE)
        self.assertEqual(ev, "cruise_resume")

    def test_target_above_cruise_stays_cruise(self):
        state, ev = csl_decide(CRUISE, 30.0, self.p)
        self.assertEqual(state, CRUISE)

    def test_illegal_state_raises(self):
        with self.assertRaises(ValueError):
            csl_decide("braking", 5.0, self.p)


class TestRampSpeed(unittest.TestCase):

    def setUp(self):
        self.p = CSLParams()

    def test_decel_limited(self):
        # 8→5 差 3，dt=0.05 → 最多减 0.1
        v = ramp_speed(8.0, 5.0, 0.05, self.p)
        self.assertAlmostEqual(v, 7.9, delta=1e-9)

    def test_accel_limited(self):
        v = ramp_speed(5.0, 8.0, 0.05, self.p)
        self.assertAlmostEqual(v, 5.075, delta=1e-9)

    def test_reaches_target_exactly(self):
        v = ramp_speed(5.2, 5.0, 0.05, self.p)  # 差 0.2 < 步长 0.1×2
        for _ in range(5):
            v = ramp_speed(v, 5.0, 0.05, self.p)
        self.assertAlmostEqual(v, 5.0)


class TestCSLController(unittest.TestCase):

    def test_event_emitted_once_on_transition(self):
        c = CSLController()
        events = []
        for _ in range(5):
            r = c.tick(5.0)   # 持续急弯目标
            events.append(r["event"])
        self.assertEqual(events[0], "slowdown_warn")
        self.assertTrue(all(e == "" for e in events[1:]))

    def test_ramp_does_not_jump(self):
        c = CSLController()
        r = c.tick(5.0)
        # 单拍后 commanded 应从 8.0 最多降到 8.0 - 2.0*0.05 = 7.9
        self.assertAlmostEqual(r["commanded_speed"], 7.9, delta=1e-9)

    def test_ramp_converges_to_target(self):
        c = CSLController()
        v = None
        for _ in range(60):
            v = c.tick(5.0)["commanded_speed"]
        self.assertAlmostEqual(v, 5.0, delta=0.05)

    def test_recovery_event(self):
        c = CSLController()
        for _ in range(50):
            c.tick(5.0)
        r = c.tick(7.95)   # 差 0.05 < slow_exit
        self.assertEqual(r["state"], CRUISE)
        self.assertEqual(r["event"], "cruise_resume")

    def test_v_min_floor(self):
        c = CSLController()
        # 半径 1m 的极急弯：理论速度 sqrt(2.5)=1.58，再受 v_min=1.5 兜底
        pts = arc_points(1.0, n=40)
        t = c.target_from_points(pts)
        self.assertGreaterEqual(t, c.p.v_min)

    def test_reset(self):
        c = CSLController()
        for _ in range(20):
            c.tick(5.0)
        c.reset()
        r = c.tick(5.0)
        self.assertAlmostEqual(r["commanded_speed"], 7.9, delta=1e-9)
        self.assertEqual(r["state"], SLOWDOWN)

    def test_custom_params(self):
        p = CSLParams(cruise_speed=15.0, a_lat_max=3.0)
        c = CSLController(p)
        pts = arc_points(10.0, n=40)   # R=10, a=3.0 → v=sqrt(30)≈5.48
        t = c.target_from_points(pts)
        self.assertAlmostEqual(t, 5.48, delta=0.1)
        r = c.tick(t)
        self.assertEqual(r["state"], SLOWDOWN)   # 5.48 远低于巡航 15


if __name__ == "__main__":
    unittest.main()
