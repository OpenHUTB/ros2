# -*- coding: utf-8 -*-
"""动态障碍场与环境无关部分的单测（不加载 MuJoCo）。"""
import math

import numpy as np

from uav_dynamic_safety_guard.core.field import (Mover, safety_field,
                                                sample_endpoints)


def test_field_is_deterministic():
    a = safety_field(seed=5, n_moving=4)
    b = safety_field(seed=5, n_moving=4)
    assert len(a.static) == len(b.static) == 18
    assert np.allclose(a.mover_positions(2.3), b.mover_positions(2.3))


def test_different_seeds_differ():
    a = safety_field(seed=1, n_moving=4)
    b = safety_field(seed=2, n_moving=4)
    assert not np.allclose(a.mover_positions(0.0), b.mover_positions(0.0))


def test_mover_keeps_circular_path():
    fld = safety_field(seed=3, n_moving=3)
    for m in fld.movers:
        for t in (0.0, 2.5, 9.0):
            p = m.pos_at(t)
            r = math.hypot(p[0] - m.center[0], p[1] - m.center[1])
            assert r == pytest_approx(m.radius)


def pytest_approx(v, tol=1e-9):
    import pytest
    return pytest.approx(v, abs=tol)


def test_mover_height_keeps_lidar_returns():
    fld = safety_field(seed=4, n_moving=6)
    for m in fld.movers:
        assert m.size[1] * 2.0 >= 1.2      # 柱高 ≥1.2 m，三层雷达都能打到


def test_speeds_within_configured_range():
    fld = safety_field(seed=9, n_moving=6)
    s = fld.speeds()
    assert s.min() >= 0.4 - 1e-9
    assert s.max() <= 1.0 + 1e-9


def test_batch_changes_with_time_but_shape_is_stable():
    fld = safety_field(seed=1, n_moving=5)
    b0 = fld.batch_at(0.0)
    b1 = fld.batch_at(2.0)
    assert b0["cyl_pos"].shape == b1["cyl_pos"].shape
    assert not np.allclose(b0["cyl_pos"], b1["cyl_pos"])


def test_clearance_smaller_next_to_mover():
    fld = safety_field(seed=7, n_moving=4)
    m = fld.movers[0]
    p = m.pos_at(0.3)
    near = fld.clearance_at(p, 0.3)
    far = fld.clearance_at(p + np.array([0.0, 7.0, 0.0]), 0.3)
    assert near < far


def test_endpoints_are_clear_and_far_apart():
    fld = safety_field(seed=11, n_moving=5)
    start, goal = sample_endpoints(fld, np.random.default_rng(0))
    assert fld.clearance_at(start, 0.0) > 0.5
    assert fld.clearance_at(goal, 0.0) > 0.4
    assert np.linalg.norm(goal - start) > 8.0


def test_static_only_field_has_no_movers():
    fld = safety_field(seed=6, n_moving=0)
    assert fld.n_moving() == 0
    assert fld.mover_positions(1.0).shape == (0, 3)
    assert np.allclose(fld.batch_at(0.0)["cyl_pos"], fld.batch_at(5.0)["cyl_pos"])


def test_mover_speed_helper():
    m = Mover("x", (0.0, 0.0), 2.0, 0.5, 0.0, (0.2, 0.6))
    assert abs(m.speed() - 1.0) < 1e-9
