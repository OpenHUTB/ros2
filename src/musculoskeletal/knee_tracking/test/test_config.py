"""配置测试：默认值、校验、非法输入。"""

from __future__ import annotations

import dataclasses

import pytest

from mujoco_musculoskeletal_knee_tracking.config import load_config, validate


def test_default_values():
    cfg = load_config()
    assert cfg.sim.dt == pytest.approx(0.005)
    assert cfg.ctrl.kp == pytest.approx(30.0)
    assert cfg.ctrl.max_torque == pytest.approx(17.5)
    assert cfg.muscle.f_max * cfg.muscle.moment_arm == pytest.approx(17.5)


def test_rejects_bad_dt():
    cfg = load_config()
    bad = dataclasses.replace(cfg, sim=dataclasses.replace(cfg.sim, dt=0.0))
    with pytest.raises(ValueError):
        validate(bad)


def test_rejects_bad_trajectory_type():
    cfg = load_config()
    bad = dataclasses.replace(
        cfg, trajectory=dataclasses.replace(cfg.trajectory, type="circle"))
    with pytest.raises(ValueError):
        validate(bad)


def test_rejects_trajectory_out_of_limits():
    cfg = load_config()
    bad = dataclasses.replace(
        cfg, trajectory=dataclasses.replace(cfg.trajectory, amplitude_deg=65.0))
    with pytest.raises(ValueError):
        validate(bad)


def test_rejects_inconsistent_torque():
    cfg = load_config()
    bad = dataclasses.replace(cfg, ctrl=dataclasses.replace(cfg.ctrl, max_torque=5.0))
    with pytest.raises(ValueError):
        validate(bad)
