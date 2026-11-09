#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双足运动学测试。"""

import numpy as np

from biped_cubic_spline_gait_switch.config_loader import (
    build_model, default_params_path, load_config)


def make_model():
    return build_model(load_config(default_params_path()))


def test_ik_fk_roundtrip():
    model = make_model()
    for hip, ankle in [((0.0, 0.84), (0.10, 0.07)), ((0.0, 0.84), (-0.12, 0.07)),
                       ((0.0, 0.80), (0.20, 0.20))]:
        q = model.leg_ik(hip, ankle)
        fk = model.leg_fk(hip, q)
        assert np.allclose(fk["ankle"], np.array(ankle), atol=1e-4)


def test_straight_leg_has_zero_knee():
    model = make_model()
    hip = (0.0, 0.80)
    ankle = (0.0, 0.0)
    q = model.leg_ik(hip, ankle)
    assert abs(q[1]) < 0.05


def test_foot_is_flat():
    model = make_model()
    q = model.leg_ik((0.0, 0.82), (0.15, 0.07))
    fk = model.leg_fk((0.0, 0.82), q)
    assert abs(fk["heel"][1] - fk["toe"][1]) < 1e-6


def test_center_of_mass_in_reasonable_range():
    model = make_model()
    q = np.zeros(6)
    com = model.center_of_mass(0.0, 0.84, 0.0, q)
    assert 0.3 < com[1] < 1.1


def test_support_polygon_single_and_double():
    model = make_model()
    q = np.zeros(6)
    lo, hi = model.support_polygon(0.0, 0.84, 0.0, q, [True, True])
    assert lo < hi
    none_lo, none_hi = model.support_polygon(0.0, 0.84, 0.0, q, [False, False])
    assert none_lo is None and none_hi is None
