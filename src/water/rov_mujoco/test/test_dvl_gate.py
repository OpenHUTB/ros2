#!/usr/bin/env python3
"""DVL 门限回归测试（纯 numpy，不需要 ROS / MuJoCo）。

同时兼容：
    python3 test/test_dvl_gate.py
    python3 -m unittest discover -s test -p "test_dvl_gate.py"
    python3 -m pytest -q test/test_dvl_gate.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rov_mujoco.ekf_fusion import ROVEKF


def make_ekf(**kwargs):
    return ROVEKF(
        [0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        **kwargs
    )


class TestDVLGate(unittest.TestCase):
    def test_disabled_by_default(self):
        ekf = make_ekf()
        for _ in range(10):
            ekf.update_dvl([0.5, 0.0, 0.0])
        self.assertEqual(ekf.dvl_accepted, 10)
        self.assertEqual(ekf.dvl_rejected, 0)

    def test_jump_gate_rejects_spike_without_state_change(self):
        ekf = make_ekf(max_dvl_jump=0.4)
        ekf.update_dvl([0.5, 0.0, 0.0])
        p0, v0, P0 = ekf.p.copy(), ekf.v.copy(), ekf.P.copy()

        ekf.update_dvl([1.2, 0.0, 0.0])
        self.assertEqual(ekf.dvl_rejected, 1)
        self.assertEqual(ekf.dvl_accepted, 1)
        self.assertTrue(np.allclose(ekf.p, p0))
        self.assertTrue(np.allclose(ekf.v, v0))
        self.assertTrue(np.allclose(ekf.P, P0))

        ekf.update_dvl([0.5, 0.0, 0.0])
        self.assertEqual(ekf.dvl_accepted, 2)
        self.assertEqual(ekf.dvl_rejected, 1)

    def test_speed_gate(self):
        ekf = make_ekf(max_dvl_speed=1.5)
        ekf.update_dvl([2.0, 0.0, 0.0])
        self.assertEqual(ekf.dvl_accepted, 0)
        self.assertEqual(ekf.dvl_rejected, 1)

    def test_non_finite(self):
        ekf = make_ekf(max_dvl_jump=1.5)
        ekf.update_dvl([float('nan'), 0.0, 0.0])
        self.assertEqual(ekf.dvl_accepted, 0)
        self.assertEqual(ekf.dvl_rejected, 1)
        self.assertEqual(ekf.dvl_rejected_invalid, 1)

    def test_jump_gate_reacquires_after_time_gap(self):
        ekf = make_ekf(max_dvl_jump=0.4, max_dvl_gap=0.5)
        ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
        ekf.update_dvl([3.0, 0.0, 0.0], timestamp=1.0)
        self.assertEqual(ekf.dvl_accepted, 2)
        self.assertEqual(ekf.dvl_rejected, 0)
        self.assertEqual(ekf.dvl_reacquired, 1)

    def test_zero_gap_disables_time_recovery(self):
        ekf = make_ekf(
            max_dvl_jump=0.4,
            max_dvl_gap=0.0,
            dvl_jump_reset_count=0)
        ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
        ekf.update_dvl([3.0, 0.0, 0.0], timestamp=10.0)
        ekf.update_dvl([3.0, 0.0, 0.0], timestamp=-5.0)
        self.assertEqual(ekf.dvl_rejected_jump, 2)
        self.assertEqual(ekf.dvl_reacquired, 0)

    def test_jump_gate_reacquires_after_reject_streak(self):
        ekf = make_ekf(max_dvl_jump=0.4, dvl_jump_reset_count=3)
        ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
        for stamp in (0.1, 0.2, 0.3):
            ekf.update_dvl([3.0, 0.0, 0.0], timestamp=stamp)
        self.assertEqual(ekf.dvl_rejected_jump, 3)
        self.assertEqual(ekf.dvl_reacquired, 1)

        ekf.update_dvl([3.0, 0.0, 0.0], timestamp=0.4)
        self.assertEqual(ekf.dvl_accepted, 2)
        self.assertEqual(ekf.dvl_rejected_jump, 3)

    def test_reject_streak_works_at_low_dvl_rates(self):
        for rate_hz in (0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0):
            with self.subTest(rate_hz=rate_hz):
                ekf = make_ekf(
                    max_dvl_jump=0.4,
                    max_dvl_gap=2.0,
                    dvl_jump_reset_count=10)
                ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
                for i in range(1, 21):
                    ekf.update_dvl(
                        [5.0, 0.0, 0.0], timestamp=i / rate_hz)
                self.assertEqual(ekf.dvl_rejected_jump, 10)
                self.assertEqual(ekf.dvl_reacquired, 1)
                self.assertEqual(ekf.dvl_accepted, 11)

    def test_invalid_samples_do_not_clear_jump_streak(self):
        ekf = make_ekf(
            max_dvl_jump=0.4,
            max_dvl_gap=2.0,
            dvl_jump_reset_count=5)
        ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
        for i in range(1, 21):
            value = [5.0, 0.0, 0.0] if i % 2 else [float('nan'), 0.0, 0.0]
            ekf.update_dvl(value, timestamp=i * 0.1)
        self.assertEqual(ekf.dvl_rejected_jump, 5)
        self.assertEqual(ekf.dvl_reacquired, 1)
        self.assertGreater(ekf.dvl_accepted, 1)

    def test_speed_gate_is_not_bypassed_by_reject_streak(self):
        ekf = make_ekf(max_dvl_speed=1.5, dvl_jump_reset_count=3)
        for i in range(20):
            ekf.update_dvl([1.8, 0.0, 0.0], timestamp=i * 0.1)
        self.assertEqual(ekf.dvl_accepted, 0)
        self.assertEqual(ekf.dvl_rejected_speed, 20)
        self.assertEqual(ekf.dvl_reacquired, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
