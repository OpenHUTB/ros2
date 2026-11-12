# -*- coding: utf-8 -*-
"""Hill 肌肉模型单元测试。"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from muscle_model import (
    MuscleParams,
    FatigueParams,
    activation_dynamics,
    force_length_active,
    force_velocity,
    muscle_force,
    fatigue_dynamics,
)


def test_force_length_peak_at_optimal():
    assert abs(force_length_active(1.0) - 1.0) < 1e-9
    assert force_length_active(1.5) < 1.0
    assert force_length_active(0.5) < 1.0


def test_force_velocity_shape():
    assert abs(force_velocity(0.0) - 1.0) < 1e-9
    assert force_velocity(-0.5) > 1.0          # 离心收缩更有力
    assert force_velocity(1.0) < 1.0           # 向心收缩衰减


def test_activation_clip():
    assert activation_dynamics(1.0, 0.0, 0.001) <= 1.0
    assert activation_dynamics(0.0, 1.0, 0.001) >= 0.0


def test_muscle_force_at_optimal_isometric():
    p = MuscleParams()
    # 最优长度、零速度、满激活：F = F_max * (1*1*1 + 0)
    assert abs(muscle_force(1.0, 1.0, 0.0, p) - p.f_max) < 1e-6


def test_fatigue_decreases_under_load():
    fp = FatigueParams()
    phi = 1.0
    for _ in range(2000):
        phi = fatigue_dynamics(0.8, phi, 0.01, fp)
    assert phi < 1.0
    assert phi >= fp.phi_min - 1e-6


def test_fatigue_recovers_under_rest_vs_load():
    # 同样 30 s：持续负荷下 phi 持续下降，休息下 phi 几乎回到 fresh
    fp = FatigueParams()
    phi_load = 1.0
    phi_rest = 1.0
    for _ in range(3000):
        phi_load = fatigue_dynamics(0.9, phi_load, 0.01, fp)
        phi_rest = fatigue_dynamics(0.0, phi_rest, 0.01, fp)
    assert phi_rest > phi_load + 0.2
    assert phi_load < 0.8


def test_force_velocity_never_negative():
    """缩短速度超过上限后力必须停在 0：收缩元件只能拉，不能推。"""
    assert force_velocity(1.0) == 0.0
    for v_n in (1.0, 1.2, 1.5, 3.0, 10.0):
        assert force_velocity(v_n) >= 0.0
    # 总力同样不能被负的力-速度因子带成推力
    params = MuscleParams()
    for v_n in (1.2, 1.5):
        assert muscle_force(1.0, 1.0, v_n, params) >= 0.0


if __name__ == "__main__":
    for name in list(globals()):
        if name.startswith("test_"):
            globals()[name]()
            print("OK:", name)
    print("全部通过")
