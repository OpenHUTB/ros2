"""RLW 决策核心离线单元测试（不依赖 ROS 与 CARLA）。

运行：python3 main.py test
"""

import unittest

from carla_red_light_warning.red_light_core import (
    APPROACHING, NORMAL, VIOLATION_RISK, RLWController, RLWParams,
    rlw_decide, stopping_distance,
)


class TestStoppingDistance(unittest.TestCase):

    def test_formula(self):
        # v=10, a=2.5 → 100/5 = 20 m
        self.assertAlmostEqual(stopping_distance(10, 2.5), 20.0)

    def test_zero_speed(self):
        self.assertEqual(stopping_distance(0, 2.5), 0.0)

    def test_negative_speed_raises(self):
        with self.assertRaises(ValueError):
            stopping_distance(-1)


class TestRLWDecide(unittest.TestCase):

    def setUp(self):
        self.p = RLWParams()   # warn_enter=30, warn_exit=35, brake_margin=1.2

    def test_green_stays_normal(self):
        state, ev = rlw_decide(NORMAL, "green", 5.0, 10.0, self.p)
        self.assertEqual(state, NORMAL)
        self.assertEqual(ev, "")

    def test_red_far_stays_normal(self):
        # 红灯但距离远（> warn_enter 30）
        state, ev = rlw_decide(NORMAL, "red", 50.0, 10.0, self.p)
        self.assertEqual(state, NORMAL)
        self.assertEqual(ev, "")

    def test_yellow_enter_approaching(self):
        state, ev = rlw_decide(NORMAL, "yellow", 20.0, 10.0, self.p)
        self.assertEqual(state, APPROACHING)
        self.assertEqual(ev, "approach_start")

    def test_yellow_never_violation(self):
        # 黄灯即使刹停不可行也只 approaching（黄灯越线不违规）
        state, ev = rlw_decide(NORMAL, "yellow", 5.0, 10.0, self.p)
        self.assertEqual(state, APPROACHING)
        self.assertEqual(ev, "approach_start")

    def test_red_unstoppable_to_violation(self):
        # 红灯、刹停不可行（d_stop=20 > dist=15）→ 直接 violation_risk
        state, ev = rlw_decide(NORMAL, "red", 15.0, 10.0, self.p)
        self.assertEqual(state, VIOLATION_RISK)
        self.assertEqual(ev, "violation_start")

    def test_approaching_to_violation(self):
        state, ev = rlw_decide(APPROACHING, "red", 15.0, 10.0, self.p)
        self.assertEqual(state, VIOLATION_RISK)
        self.assertEqual(ev, "violation_start")

    def test_approaching_clear_by_green(self):
        state, ev = rlw_decide(APPROACHING, "green", 20.0, 10.0, self.p)
        self.assertEqual(state, NORMAL)
        self.assertEqual(ev, "approach_clear")

    def test_approaching_hysteresis(self):
        # dist=32 在 warn_enter(30) 与 warn_exit(35) 之间 → 保持
        state, ev = rlw_decide(APPROACHING, "yellow", 32.0, 10.0, self.p)
        self.assertEqual(state, APPROACHING)
        self.assertEqual(ev, "")
        # > 35 → 恢复 normal
        state, ev = rlw_decide(APPROACHING, "yellow", 36.0, 10.0, self.p)
        self.assertEqual(state, NORMAL)
        self.assertEqual(ev, "approach_clear")

    def test_violation_hysteresis(self):
        # d_stop=20，brake_exit=24；dist=22 在两者之间 → 保持 violation
        state, ev = rlw_decide(VIOLATION_RISK, "red", 22.0, 10.0, self.p)
        self.assertEqual(state, VIOLATION_RISK)
        self.assertEqual(ev, "")
        # > 24 → 回退 approaching（发 violation_clear）
        state, ev = rlw_decide(VIOLATION_RISK, "red", 25.0, 10.0, self.p)
        self.assertEqual(state, APPROACHING)
        self.assertEqual(ev, "violation_clear")

    def test_violation_cleared_by_green(self):
        state, ev = rlw_decide(VIOLATION_RISK, "green", 10.0, 10.0, self.p)
        self.assertEqual(state, NORMAL)
        self.assertEqual(ev, "violation_clear")

    def test_slow_speed_shrinks_stop_distance(self):
        # 低速下刹停距离小：红灯 8 m 处 v=3（d_stop=1.8）→ 不 violation，approaching
        state, ev = rlw_decide(NORMAL, "red", 8.0, 3.0, self.p)
        self.assertEqual(state, APPROACHING)

    def test_invalid_light_raises(self):
        with self.assertRaises(ValueError):
            rlw_decide(NORMAL, "blue", 10.0, 5.0, self.p)

    def test_invalid_state_raises(self):
        with self.assertRaises(ValueError):
            rlw_decide("braking", "red", 10.0, 5.0, self.p)


class TestRLWController(unittest.TestCase):

    def test_full_cycle_scenario(self):
        """绿灯→红灯接近→将闯线→绿灯，完整事件链。"""
        c = RLWController()
        seq = [("green", 100, 8), ("green", 90, 8),      # 热身 2 拍
               ("red", 25, 8),      # approach_start
               ("red", 20, 8),      # 保持（d_stop=12.8<20）
               ("red", 12, 8),      # violation_start（12<12.8）
               ("red", 12, 8),
               ("green", 10, 8),    # violation_clear
               ("green", 10, 8)]
        events = []
        states = []
        for light, d, v in seq:
            r = c.tick(light, d, v)
            states.append(r["state"])
            if r["event"]:
                events.append(r["event"])
        self.assertEqual(events, ["approach_start", "violation_start",
                                  "violation_clear"])
        self.assertEqual(states[-1], NORMAL)

    def test_warmup_suppresses_cold_start(self):
        """热身期内不输出决策（冷启动灯态未稳定）。"""
        c = RLWController()
        r = c.tick("red", 5.0, 10.0)   # 热身拍：本应直接 violation
        self.assertEqual(r["state"], NORMAL)
        self.assertEqual(r["event"], "")
        r = c.tick("red", 5.0, 10.0)
        self.assertEqual(r["state"], NORMAL)
        r = c.tick("red", 5.0, 10.0)   # 热身结束，正常决策
        self.assertEqual(r["state"], VIOLATION_RISK)
        self.assertEqual(r["event"], "violation_start")

    def test_warmup_zero_disables(self):
        c = RLWController(warmup_ticks=0)
        r = c.tick("red", 5.0, 10.0)
        self.assertEqual(r["state"], VIOLATION_RISK)

    def test_event_emitted_once_only(self):
        c = RLWController()
        c.tick("green", 100, 8)
        c.tick("green", 100, 8)
        events = [c.tick("red", 25, 8)["event"] for _ in range(5)]
        self.assertEqual(events[0], "approach_start")
        self.assertTrue(all(e == "" for e in events[1:]))

    def test_reset_restarts_warmup(self):
        c = RLWController()
        c.tick("green", 100, 8)
        c.tick("green", 100, 8)
        c.tick("red", 25, 8)
        c.reset()
        r = c.tick("red", 5.0, 10.0)   # 复位后重新热身，不报警
        self.assertEqual(r["state"], NORMAL)
        self.assertEqual(r["event"], "")

    def test_invalid_light_in_tick_raises(self):
        c = RLWController()
        with self.assertRaises(ValueError):
            c.tick("purple", 10.0, 5.0)


if __name__ == "__main__":
    unittest.main()
