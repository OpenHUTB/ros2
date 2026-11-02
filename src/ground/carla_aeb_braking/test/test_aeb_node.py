"""离线单元测试：AEB 控制节点（无 ROS 运行环境也可跑）。

节点顶层 import rclpy，宿主机（无 ROS）不能直接 import，
因此节点行为用“源码守卫”测试（与 carla_perception_control 同一惯例）；
节点依赖的纯逻辑（estimate_closing / abort / reset）直接测试。
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from carla_aeb_braking.aeb_core import (  # noqa: E402
    BRAKING,
    CRUISE,
    AEBController,
    AEBParams,
    estimate_closing,
)


def _node_source():
    return (ROOT / "carla_aeb_braking" / "aeb_node.py").read_text(encoding="utf-8")


class TestEstimateClosing(unittest.TestCase):
    def test_closing_in(self):
        self.assertAlmostEqual(estimate_closing(20.0, 19.0, 0.5), 2.0)

    def test_moving_away_negative(self):
        self.assertAlmostEqual(estimate_closing(10.0, 12.0, 0.5), -4.0)

    def test_first_sample_fallback(self):
        self.assertAlmostEqual(estimate_closing(None, 19.0, 0.5, fallback=12.0), 12.0)

    def test_bad_dt_fallback(self):
        self.assertAlmostEqual(estimate_closing(20.0, 19.0, 0.0, fallback=5.0), 5.0)


class TestAbortReset(unittest.TestCase):
    def test_abort_from_braking(self):
        ctl = AEBController()
        ctl.tick(5.0, 5.0)  # → BRAKING
        self.assertEqual(ctl.state, BRAKING)
        ctl.abort()
        self.assertEqual(ctl.state, CRUISE)
        # 障碍已远：保持巡航、无制动
        _, throttle, brake, _ = ctl.tick(50.0, 0.0)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)

    def test_abort_with_obstacle_still_close_rewarns(self):
        ctl = AEBController()
        ctl.tick(5.0, 5.0)   # BRAKING
        ctl.abort()          # → CRUISE
        state, _, _, event = ctl.tick(15.0, 1.0)
        self.assertEqual(state, "warning")
        self.assertEqual(event, "warn")

    def test_reset_clears_history(self):
        ctl = AEBController()
        ctl.tick(50.0, 0.0)
        ctl.tick(5.0, 5.0)
        ctl.reset()
        self.assertEqual(ctl.state, CRUISE)
        self.assertEqual(ctl.history, [])
        self.assertEqual(ctl.events, [])


class TestAebNodeSourceGuards(unittest.TestCase):
    """控制节点关键行为守卫（源码级，避免依赖 ROS 运行环境）。"""

    def test_subscribes_distance_and_speed(self):
        src = _node_source()
        self.assertIn('"/aeb/obstacle_distance"', src)
        self.assertIn('"/aeb/ego_speed"', src)

    def test_publishes_control_state_event(self):
        src = _node_source()
        self.assertIn('"/aeb/control"', src)
        self.assertIn('"/aeb/state"', src)
        self.assertIn('"/aeb/event"', src)
        self.assertIn("Float32MultiArray", src)

    def test_arm_abort_reset_services(self):
        src = _node_source()
        for srv in ('"/aeb/arm"', '"/aeb/abort"', '"/aeb/reset"'):
            self.assertIn(srv, src)

    def test_uses_state_machine_and_closing_estimate(self):
        src = _node_source()
        self.assertIn("self.ctl.tick(", src)
        self.assertIn("estimate_closing(", src)


if __name__ == "__main__":
    unittest.main()
