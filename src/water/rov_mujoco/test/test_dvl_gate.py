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
    print('PASS non-finite measurement rejected')


if __name__ == '__main__':
    test_disabled_by_default()
    test_jump_gate_rejects_spike_without_state_change()
    test_speed_gate()
    test_non_finite()
    print('all DVL gate regression tests passed')
