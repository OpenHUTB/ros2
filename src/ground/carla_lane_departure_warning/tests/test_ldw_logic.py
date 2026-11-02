"""LDW 决策核心离线单元测试（不依赖 ROS 与 CARLA）。

运行：python3 main.py test
"""

import math
import unittest

from carla_lane_departure_warning.lane_departure_core import (
    DEPARTURE, DEVIATION, NORMAL, LDWController, LDWParams,
    heading_error, lateral_offset, ldw_decide, normalize_angle,
    reject_jump, side_of,
)


class TestLateralOffset(unittest.TestCase):

    def test_on_centerline_zero(self):
        # 自车就在中心线上 → 偏移 0
        self.assertEqual(lateral_offset((10, 0), (10, 0), 0.0), 0.0)

    def test_left_positive(self):
        # 道路朝 +x，自车偏左（+y）→ 正
        self.assertAlmostEqual(lateral_offset((10, 0.8), (10, 0), 0.0), 0.8)

    def test_right_negative(self):
        self.assertAlmostEqual(lateral_offset((10, -0.8), (10, 0), 0.0), -0.8)

    def test_rotated_road(self):
        # 道路朝 +y（yaw=π/2），自车偏"左"即 −x 方向 → 正
        o = lateral_offset((9.2, 5), (10, 5), math.pi / 2)
        self.assertAlmostEqual(o, 0.8, delta=1e-9)

    def test_longitudinal_displacement_no_effect(self):
        # 沿道路方向的前后错位不影响横向偏移
        o = lateral_offset((14, 0.5), (10, 0), 0.0)
        self.assertAlmostEqual(o, 0.5)


class TestHeadingError(unittest.TestCase):

    def test_aligned_zero(self):
        self.assertAlmostEqual(heading_error(0.3, 0.3), 0.0)

    def test_left_positive(self):
        self.assertAlmostEqual(heading_error(0.5, 0.0), 0.5)

    def test_wraps_to_pi_range(self):
        # (π−0.1) − (−π+0.1) = 2π−0.2 ≡ −0.2 (mod 2π)
        e = heading_error(math.pi - 0.1, -math.pi + 0.1)
        self.assertAlmostEqual(e, -0.2, delta=1e-9)
        self.assertLessEqual(abs(e), math.pi)

    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(3 * math.pi), math.pi, delta=1e-9)


class TestSideOf(unittest.TestCase):

    def test_sides(self):
        self.assertEqual(side_of(0.2), "left")
        self.assertEqual(side_of(-0.2), "right")
        self.assertEqual(side_of(0.0), "")


class TestRejectJump(unittest.TestCase):

    def test_first_pass_through(self):
        cur, rej = reject_jump(None, 0.8)
        self.assertEqual(cur, 0.8)
        self.assertFalse(rej)

    def test_normal_change_passes(self):
        cur, rej = reject_jump(0.5, 0.8)   # 步长 0.3 < 1.5
        self.assertEqual(cur, 0.8)
        self.assertFalse(rej)

    def test_jump_limited_to_step(self):
        cur, rej = reject_jump(0.41, -1.50)   # 跳变 1.91 → 限幅 1.5
        self.assertAlmostEqual(cur, -1.09, delta=1e-9)
        self.assertTrue(rej)

    def test_jump_direction_both_sides(self):
        cur, _ = reject_jump(-1.0, 2.0)   # +3.0 → -1.0+1.5 = 0.5
        self.assertAlmostEqual(cur, 0.5)
        cur, _ = reject_jump(1.0, -2.0)   # -3.0 → 1.0-1.5 = -0.5
        self.assertAlmostEqual(cur, -0.5)


class TestLDWDecide(unittest.TestCase):

    def setUp(self):
        self.p = LDWParams()

    def test_normal_stays_within_warn(self):
        state, ev = ldw_decide(NORMAL, 0.4, self.p)
        self.assertEqual(state, NORMAL)
        self.assertEqual(ev, "")

    def test_normal_to_deviation(self):
        state, ev = ldw_decide(NORMAL, 0.6, self.p)
        self.assertEqual(state, DEVIATION)
        self.assertEqual(ev, "deviation_start_left")

    def test_normal_to_departure_direct(self):
        # 一拍跨过 deviation 直达 departure
        state, ev = ldw_decide(NORMAL, -1.2, self.p)
        self.assertEqual(state, DEPARTURE)
        self.assertEqual(ev, "departure_start_right")

    def test_deviation_to_departure(self):
        state, ev = ldw_decide(DEVIATION, 1.0, self.p)
        self.assertEqual(state, DEPARTURE)
        self.assertEqual(ev, "departure_start_left")

    def test_deviation_hysteresis_no_flap(self):
        # 0.4 在 warn_exit(0.3) 与 warn_enter(0.5) 之间 → 保持 deviation
        state, ev = ldw_decide(DEVIATION, 0.4, self.p)
        self.assertEqual(state, DEVIATION)
        self.assertEqual(ev, "")
        # < 0.3 → 恢复 normal
        state, ev = ldw_decide(DEVIATION, 0.2, self.p)
        self.assertEqual(state, NORMAL)
        self.assertEqual(ev, "deviation_clear")

    def test_departure_hysteresis(self):
        # 0.75 在 depart_exit(0.65) 与 depart_enter(0.9) 之间 → 保持 departure
        state, ev = ldw_decide(DEPARTURE, 0.75, self.p)
        self.assertEqual(state, DEPARTURE)
        self.assertEqual(ev, "")
        # < 0.65 → 回 deviation（发 departure_end，不带方向）
        state, ev = ldw_decide(DEPARTURE, 0.6, self.p)
        self.assertEqual(state, DEVIATION)
        self.assertEqual(ev, "departure_end")

    def test_event_emitted_once_only(self):
        c = LDWController()
        events = [c.tick(0.8)["event"] for _ in range(5)]
        self.assertEqual(events[0], "deviation_start_left")
        self.assertTrue(all(e == "" for e in events[1:]))

    def test_illegal_state_raises(self):
        with self.assertRaises(ValueError):
            ldw_decide("braking", 0.5, self.p)


class TestLDWController(unittest.TestCase):

    def test_full_cycle_scenario(self):
        """正常 → 左偏离预警 → 压线 → 回中，完整事件链。"""
        c = LDWController()
        seq = [0.0, 0.3, 0.6, 0.8, 1.0, 1.2, 0.8, 0.6, 0.2, 0.0]
        events = []
        states = []
        for o in seq:
            r = c.tick(o)
            states.append(r["state"])
            if r["event"]:
                events.append(r["event"])
        self.assertEqual(events, ["deviation_start_left", "departure_start_left",
                                  "departure_end", "deviation_clear"])
        self.assertEqual(states[-1], NORMAL)

    def test_right_side_events(self):
        c = LDWController()
        r = c.tick(-0.7)
        self.assertEqual(r["event"], "deviation_start_right")
        self.assertEqual(r["side"], "right")

    def test_reset(self):
        c = LDWController()
        c.tick(1.2)
        c.reset()
        r = c.tick(0.0)
        self.assertEqual(r["state"], NORMAL)

    def test_custom_params(self):
        p = LDWParams(warn_enter=0.2, depart_enter=0.4)
        state, ev = ldw_decide(NORMAL, 0.3, p)
        self.assertEqual(state, DEVIATION)
        self.assertEqual(ev, "deviation_start_left")


if __name__ == "__main__":
    unittest.main()
