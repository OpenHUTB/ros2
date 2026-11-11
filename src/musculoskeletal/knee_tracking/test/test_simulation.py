"""仿真与指标测试：正弦/阶跃形状、指标定义、与文档数量级一致。"""

from __future__ import annotations

import numpy as np
import pytest

from mujoco_musculoskeletal_knee_tracking.config import load_config
from mujoco_musculoskeletal_knee_tracking.model import KneeSim
from mujoco_musculoskeletal_knee_tracking.simulation import (
    run_tracking,
    sine_metrics,
    step_metrics,
    sine_ref,
    step_ref,
)


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def sim(cfg):
    return KneeSim(cfg)


def test_reference_functions(cfg):
    traj = cfg.trajectory
    assert sine_ref(0.0, traj.bias_deg, traj.amplitude_deg,
                    traj.frequency_hz) == pytest.approx(traj.bias_deg)
    assert step_ref(0.0, 20.0, 50.0, 0.5) == 20.0
    assert step_ref(0.6, 20.0, 50.0, 0.5) == 50.0


def test_sine_run_shape(cfg, sim):
    run = run_tracking(sim, cfg, "sine", duration=2.0)
    assert run.t.shape == run.theta_deg.shape == run.ref_deg.shape
    assert np.all(np.isfinite(run.theta_deg))
    assert len(run.t) == int(round(2.0 / cfg.sim.dt)) + 1


def test_sine_tracking_quality(cfg, sim):
    run = run_tracking(sim, cfg, "sine", duration=10.0,
                       initial_deg=cfg.trajectory.bias_deg)
    m = sine_metrics(run)
    # 文档：RMSE 8.65°，最大 13.96°；允许 20% 偏差
    assert 6.0 < m["rmse_deg"] < 10.5
    assert 10.0 < m["max_abs_error_deg"] < 18.0


def test_step_steady_error_matches_document(cfg, sim):
    run = run_tracking(sim, cfg, "step", duration=8.0,
                       initial_deg=cfg.trajectory.step_start_deg)
    m = step_metrics(run, cfg)
    # 重力未前馈 -> 稳态下垂约 -6.62°
    assert m["steady_state_error_deg"] == pytest.approx(-6.62, abs=0.6)
    assert m["overshoot_percent"] < 2.0


def test_metrics_are_finite(cfg, sim):
    run = run_tracking(sim, cfg, "step", duration=3.0,
                       initial_deg=cfg.trajectory.step_start_deg)
    for value in step_metrics(run, cfg).values():
        assert value is None or np.isfinite(value)


def test_step_metrics_definitions(cfg):
    from mujoco_musculoskeletal_knee_tracking.simulation import Run

    dt = cfg.sim.dt
    n = int(8.0 / dt) + 1
    t = np.arange(n) * dt
    target, start = cfg.trajectory.step_target_deg, cfg.trajectory.step_start_deg
    ref = np.where(t < cfg.trajectory.step_time, start, target)
    theta = np.where(t < cfg.trajectory.step_time, start, target - 0.1)
    run = Run(t=t, theta_deg=theta, ref_deg=ref, error_deg=theta - ref,
              flex=np.zeros(n), ext=np.zeros(n), torque=np.zeros(n))
    m = step_metrics(run, cfg)
    assert m["steady_state_error_deg"] == pytest.approx(-0.1, abs=0.02)
    assert m["overshoot_percent"] == pytest.approx(0.0, abs=1e-9)
    assert m["settling_time_s"] is not None
