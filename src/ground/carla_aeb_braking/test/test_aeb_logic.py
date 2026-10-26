"""离线单元测试：AEB 状态机（进入/退出/滞回/全周期）。"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from carla_aeb_braking.aeb_core import (  # noqa: E402
    BRAKING,
    CRUISE,
    RECOVER,
    WARNING,
    AEBController,
    AEBParams,
    aeb_decide,
)

P = AEBParams()


class TestAebDecide(unittest.TestCase):
    def test_cruise_when_far(self):
        state, throttle, brake, event = aeb_decide(CRUISE, d=50.0, closing=5.0, p=P)
        self.assertEqual(state, CRUISE)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)
        self.assertIsNone(event)

    def test_warn_enter_from_cruise(self):
        state, _, _, event = aeb_decide(CRUISE, d=15.0, closing=2.0, p=P)
        self.assertEqual(state, WARNING)
        self.assertEqual(event, "warn")

    def test_brake_enter_when_close_and_closing(self):
        for start in (CRUISE, WARNING):
            state, throttle, brake, event = aeb_decide(start, d=5.0, closing=3.0, p=P)
            self.assertEqual(state, BRAKING)
            self.assertEqual(throttle, 0.0)
            self.assertEqual(brake, P.brake_max)
            self.assertEqual(event, "brake")

    def test_no_brake_when_not_closing(self):
        state, _, brake, _ = aeb_decide(CRUISE, d=5.0, closing=0.0, p=P)
        self.assertNotEqual(state, BRAKING)
        self.assertLess(brake, P.brake_max)

    def test_brake_holds_while_close(self):
        state, _, brake, event = aeb_decide(BRAKING, d=8.0, closing=3.0, p=P)
        self.assertEqual(state, BRAKING)
        self.assertEqual(brake, P.brake_max)
        self.assertIsNone(event)

    def test_brake_exit_to_recover(self):
        state, _, _, event = aeb_decide(BRAKING, d=15.0, closing=0.0, p=P)
        self.assertEqual(state, RECOVER)
        self.assertEqual(event, "recover")

    def test_recover_to_cruise_when_far(self):
        state, throttle, brake, _ = aeb_decide(RECOVER, d=30.0, closing=0.0, p=P)
        self.assertEqual(state, CRUISE)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)

    def test_warn_hysteresis_no_chatter(self):
        # 滞回带 (warn_enter, warn_exit]：WARNING 中距离回升到带内不应退回 CRUISE
        state, _, _, _ = aeb_decide(CRUISE, d=17.0, closing=1.0, p=P)
        self.assertEqual(state, WARNING)
        state, _, _, _ = aeb_decide(WARNING, d=20.0, closing=0.0, p=P)
        self.assertEqual(state, WARNING)          # 20 在滞回带内，保持预警
        state, _, _, _ = aeb_decide(WARNING, d=25.0, closing=0.0, p=P)
        self.assertEqual(state, CRUISE)           # 越过 warn_exit 才回巡航
        # CRUISE 中距离在滞回带内也不应反复进入预警
        state, _, _, _ = aeb_decide(CRUISE, d=20.0, closing=1.0, p=P)
        self.assertEqual(state, CRUISE)

    def test_bad_state_raises(self):
        with self.assertRaises(ValueError):
            aeb_decide("bogus", d=10.0, closing=1.0, p=P)

    def test_custom_params_respected(self):
        p = AEBParams(brake_enter=2.0, close_min=5.0)
        state, _, _, _ = aeb_decide(CRUISE, d=5.0, closing=3.0, p=p)
        self.assertNotEqual(state, BRAKING)       # 5 > brake_enter=2
        state, _, _, _ = aeb_decide(CRUISE, d=1.0, closing=4.0, p=p)
        self.assertNotEqual(state, BRAKING)       # 4 < close_min=5
        state, _, _, _ = aeb_decide(CRUISE, d=1.0, closing=6.0, p=p)
        self.assertEqual(state, BRAKING)


class TestFullCycle(unittest.TestCase):
    def test_far_to_close_to_clear(self):
        ctl = AEBController()
        seen = [ctl.state]
        d = 60.0
        for t in range(120):
            # 前 14 tick 障碍物以 10 m/s 逼近，之后以 8 m/s 驶离（closing 取负）
            closing = 10.0 if t < 14 else -8.0
            d = max(d - closing * 0.5, 0.5)
            state, throttle, brake, _ = ctl.tick(d, closing, t=t * 0.5)
            seen.append(state)
            if state == CRUISE and t > 30:  # 障碍驶离后的稳态
                break
        order = [s for s in dict.fromkeys(seen)]  # 去重保序（首尾同为 cruise，去重后 4 态）
        self.assertEqual(order, [CRUISE, WARNING, BRAKING, RECOVER])
        # 恢复之后确实回到了巡航（驶离后收尾）
        i_recover = next(i for i, s in enumerate(seen) if s == RECOVER)
        self.assertEqual(seen[-1], CRUISE)
        self.assertIn(CRUISE, seen[i_recover + 1:])
        # 最近距离阶段必须在全力制动
        nearest = min(ctl.history, key=lambda h: h[2])
        self.assertEqual(nearest[5], P.brake_max)
        self.assertIn("brake", [e for _, e in ctl.events])


if __name__ == "__main__":
    unittest.main()
