# -*- coding: utf-8 -*-
"""候选集与安全层选择的单测（无需仿真）。"""
import numpy as np
import pytest

from uav_dynamic_safety_guard.core import ranges as R
from uav_dynamic_safety_guard.core.filter import (BrakeFilter, FilterConfig,
                                                  NoFilter, candidates, select)


def test_candidates_are_unique_and_bounded():
    cfg = FilterConfig()
    c = candidates(np.array([2.0, 0.0, 0.0]), cfg)
    assert len(c) > 60
    assert np.max(np.abs(c[:, 0])) <= cfg.max_horiz + 1e-9
    assert np.max(np.abs(c[:, 1])) <= 3.0 + 1e-9
    assert np.max(np.abs(c[:, 2])) <= cfg.max_vert + 1e-9
    assert len(np.unique(np.round(c, 3), axis=0)) == len(c)


def test_candidates_include_brake_and_climb():
    c = candidates(np.array([2.0, 0.0, 0.0]))
    assert np.any(np.all(np.abs(c) < 1e-9, axis=1))
    assert np.any(c[:, 2] > 1.0)


def test_candidates_scale_with_nominal_speed():
    slow = candidates(np.array([0.5, 0.0, 0.0]))
    fast = candidates(np.array([3.0, 0.0, 0.0]))
    assert np.max(np.linalg.norm(fast[:, :2], axis=1)) > \
        np.max(np.linalg.norm(slow[:, :2], axis=1))


def test_select_prefers_feasible_near_nominal():
    cfg = FilterConfig(margin=0.8)
    u_nom = np.array([2.0, 0.0, 0.0])
    c = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0]])
    q = np.array([0.5, 0.9, 1.5])
    u, diag = select(u_nom, c, 0.3, q, cfg)
    assert np.allclose(u, [2.0, 0.0, 0.0])
    assert diag["n_feasible"] == 2.0


def test_select_fallback_maximizes_clearance():
    cfg = FilterConfig(margin=0.8)
    u_nom = np.array([2.0, 0.0, 0.0])
    c = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 1.0, 0.0]])
    q = np.array([0.3, 0.2, 0.7])
    u, diag = select(u_nom, c, 0.1, q, cfg)
    assert np.allclose(u, [1.0, 1.0, 0.0])
    assert diag["n_feasible"] == 0.0


def test_select_uses_goal_term_among_feasible():
    cfg = FilterConfig(margin=0.8, progress=5.0)
    u_nom = np.array([2.0, 0.0, 0.0])
    c = np.array([[1.8, 0.0, 0.0], [1.0, 1.0, 0.0]])
    q = np.array([1.0, 1.0])
    u, _ = select(u_nom, c, 1.0, q, cfg, goal_dir=np.array([0.0, 1.0, 0.0]))
    assert np.allclose(u, [1.0, 1.0, 0.0])


def test_no_filter_passes_through():
    obs = {"ranges": np.full((R.N_EL, R.N_AZ), 3.0, np.float32), "h": 1.2}
    u_nom = np.array([1.5, -0.2, 0.1])
    u, _ = NoFilter().act(obs, u_nom)
    assert np.allclose(u, u_nom)


def test_brake_filter_slows_near_obstacle():
    far = {"ranges": np.full((R.N_EL, R.N_AZ), 3.0, np.float32), "h": 1.0}
    near = {"ranges": np.full((R.N_EL, R.N_AZ), 0.5, np.float32), "h": 0.5}
    u_nom = np.array([1.5, 0.0, 0.0])
    f = BrakeFilter(stop_m=1.2)
    assert np.allclose(f.act(far, u_nom)[0], u_nom)
    assert np.linalg.norm(f.act(near, u_nom)[0]) < np.linalg.norm(u_nom)


def test_brake_filter_never_reverses():
    near = {"ranges": np.full((R.N_EL, R.N_AZ), 0.1, np.float32), "h": 0.1}
    u, _ = BrakeFilter(1.2).act(near, np.array([1.5, 0.0, 0.0]))
    assert np.all(u >= -1e-9)


def test_filter_config_defaults_are_consistent():
    cfg = FilterConfig()
    assert cfg.margin > 0 and cfg.horizon > 0
    assert cfg.max_yaw_rate > 0 and cfg.acc_max > 0
    with pytest.raises(ValueError):
        from uav_dynamic_safety_guard.core.filter import NeuralGuard
        NeuralGuard(None, mode="bogus")
