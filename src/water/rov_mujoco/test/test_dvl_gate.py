#!/usr/bin/env python3
"""DVL 门限最小回归测试（纯 numpy，不需要 ROS / MuJoCo）。

运行：
    python3 test/test_dvl_gate.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rov_mujoco.ekf_fusion import ROVEKF


def make_ekf(**kwargs):
    return ROVEKF([0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], **kwargs)


def test_disabled_by_default():
    ekf = make_ekf()
    for _ in range(10):
        ekf.update_dvl([0.5, 0.0, 0.0])
    assert ekf.dvl_accepted == 10, ekf.dvl_accepted
    assert ekf.dvl_rejected == 0, ekf.dvl_rejected
    print('PASS default thresholds disabled')


def test_jump_gate_rejects_spike_without_state_change():
    ekf = make_ekf(max_dvl_jump=0.4)          # 只开跳变门限
    ekf.update_dvl([0.5, 0.0, 0.0])
    p0, v0, P0 = ekf.p.copy(), ekf.v.copy(), ekf.P.copy()

    ekf.update_dvl([1.2, 0.0, 0.0])           # 跳变 0.7 > 0.4
    assert ekf.dvl_rejected == 1, ekf.dvl_rejected
    assert ekf.dvl_accepted == 1, ekf.dvl_accepted
    assert np.allclose(ekf.p, p0)
    assert np.allclose(ekf.v, v0)
    assert np.allclose(ekf.P, P0)

    ekf.update_dvl([0.5, 0.0, 0.0])           # 恢复正常，且未被尖峰污染基准
    assert ekf.dvl_accepted == 2, ekf.dvl_accepted
    assert ekf.dvl_rejected == 1, ekf.dvl_rejected
    print('PASS jump gate rejects spike, state unchanged, recovery accepted')


def test_speed_gate():
    ekf = make_ekf(max_dvl_speed=1.5)
    ekf.update_dvl([2.0, 0.0, 0.0])
    assert ekf.dvl_accepted == 0, ekf.dvl_accepted
    assert ekf.dvl_rejected == 1, ekf.dvl_rejected
    print('PASS speed gate rejects overspeed')


def test_non_finite():
    ekf = make_ekf(max_dvl_jump=1.5)
    ekf.update_dvl([float('nan'), 0.0, 0.0])
    assert ekf.dvl_accepted == 0, ekf.dvl_accepted
    assert ekf.dvl_rejected == 1, ekf.dvl_rejected
    assert ekf.dvl_rejected_invalid == 1, ekf.dvl_rejected_invalid
    print('PASS non-finite measurement rejected')


def test_jump_gate_reacquires_after_time_gap():
    ekf = make_ekf(max_dvl_jump=0.4, max_dvl_gap=0.5)
    ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
    ekf.update_dvl([3.0, 0.0, 0.0], timestamp=1.0)
    assert ekf.dvl_accepted == 2, ekf.dvl_accepted
    assert ekf.dvl_rejected == 0, ekf.dvl_rejected
    assert ekf.dvl_reacquired == 1, ekf.dvl_reacquired
    print('PASS jump baseline reacquired after time gap')


def test_jump_gate_reacquires_after_reject_streak():
    ekf = make_ekf(max_dvl_jump=0.4, dvl_jump_reset_count=3)
    ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
    for stamp in (0.1, 0.2, 0.3):
        ekf.update_dvl([3.0, 0.0, 0.0], timestamp=stamp)
    assert ekf.dvl_rejected_jump == 3, ekf.dvl_rejected_jump
    assert ekf.dvl_reacquired == 1, ekf.dvl_reacquired

    ekf.update_dvl([3.0, 0.0, 0.0], timestamp=0.4)
    assert ekf.dvl_accepted == 2, ekf.dvl_accepted
    assert ekf.dvl_rejected_jump == 3, ekf.dvl_rejected_jump
    print('PASS jump baseline reacquired after reject streak')


def test_reject_streak_works_at_low_dvl_rates():
    for rate_hz in (1.0, 2.0, 5.0, 10.0):
        ekf = make_ekf(
            max_dvl_jump=0.4,
            max_dvl_gap=2.0,
            dvl_jump_reset_count=10)
        ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
        for i in range(1, 21):
            ekf.update_dvl(
                [5.0, 0.0, 0.0], timestamp=i / rate_hz)
        assert ekf.dvl_rejected_jump == 10, (rate_hz, ekf.dvl_rejected_jump)
        assert ekf.dvl_reacquired == 1, (rate_hz, ekf.dvl_reacquired)
        assert ekf.dvl_accepted == 11, (rate_hz, ekf.dvl_accepted)
    print('PASS jump streak remains effective at 1-10 Hz DVL rates')


def test_invalid_samples_do_not_clear_jump_streak():
    ekf = make_ekf(
        max_dvl_jump=0.4,
        max_dvl_gap=2.0,
        dvl_jump_reset_count=5)
    ekf.update_dvl([0.5, 0.0, 0.0], timestamp=0.0)
    for i in range(1, 21):
        value = [5.0, 0.0, 0.0] if i % 2 else [float('nan'), 0.0, 0.0]
        ekf.update_dvl(value, timestamp=i * 0.1)
    assert ekf.dvl_rejected_jump == 5, ekf.dvl_rejected_jump
    assert ekf.dvl_reacquired == 1, ekf.dvl_reacquired
    assert ekf.dvl_accepted > 1, ekf.dvl_accepted
    print('PASS invalid samples do not clear jump streak')


def test_speed_gate_is_not_bypassed_by_reject_streak():
    ekf = make_ekf(max_dvl_speed=1.5, dvl_jump_reset_count=3)
    for i in range(20):
        ekf.update_dvl([1.8, 0.0, 0.0], timestamp=i * 0.1)
    assert ekf.dvl_accepted == 0, ekf.dvl_accepted
    assert ekf.dvl_rejected_speed == 20, ekf.dvl_rejected_speed
    assert ekf.dvl_reacquired == 0, ekf.dvl_reacquired
    print('PASS absolute speed gate is not bypassed by recovery')


if __name__ == '__main__':
    test_disabled_by_default()
    test_jump_gate_rejects_spike_without_state_change()
    test_speed_gate()
    test_non_finite()
    test_jump_gate_reacquires_after_time_gap()
    test_jump_gate_reacquires_after_reject_streak()
    test_reject_streak_works_at_low_dvl_rates()
    test_invalid_samples_do_not_clear_jump_streak()
    test_speed_gate_is_not_bypassed_by_reject_streak()
    print('all DVL gate regression tests passed')
