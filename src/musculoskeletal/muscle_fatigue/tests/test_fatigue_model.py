# -*- coding: utf-8 -*-
"""疲劳动力学单元测试。"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from muscle_model import (
    MuscleParams,
    FatigueParams,
    fatigue_dynamics,
    fatigued_muscle_force,
)


def test_steady_state_under_sustained_activation():
    fp = FatigueParams()
    phi = 1.0
    for _ in range(8000):
        phi = fatigue_dynamics(1.0, phi, 0.01, fp)
    # 持续满激活应趋近下界 phi_min
    assert abs(phi - fp.phi_min) < 0.02


def test_force_scales_linearly_with_phi():
    p = MuscleParams()
    f_fresh = fatigued_muscle_force(0.8, 1.0, 0.0, 1.0, p)
    f_tired = fatigued_muscle_force(0.8, 1.0, 0.0, 0.6, p)
    assert abs(f_tired - 0.6 * f_fresh) < 1e-9


def test_rest_vs_load_final_phi_differs():
    fp = FatigueParams()
    phi_load = 1.0
    phi_rest = 1.0
    for _ in range(3000):
        phi_load = fatigue_dynamics(0.9, phi_load, 0.01, fp)
        phi_rest = fatigue_dynamics(0.0, phi_rest, 0.01, fp)
    assert phi_load < phi_rest


if __name__ == "__main__":
    for name in list(globals()):
        if name.startswith("test_"):
            globals()[name]()
            print("OK:", name)
    print("全部通过")
