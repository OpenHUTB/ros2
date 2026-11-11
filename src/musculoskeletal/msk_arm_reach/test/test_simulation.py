"""闭环到达测试：默认目标 3.2 s 内收敛到 1 cm 内。"""

from __future__ import annotations

import numpy as np
import pytest

from mujoco_musculoskeletal_arm_reach.config import load_config
from mujoco_musculoskeletal_arm_reach.simulation import run_reach


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def run(cfg):
    return run_reach(cfg)


def test_shapes(run):
    assert run.t.shape == run.errors.shape
    assert run.tip.shape[1] == 2
    assert run.activations.shape[1] == 6
    assert np.all(np.isfinite(run.tip))
    assert np.all(np.isfinite(run.errors))


def test_reaches_default_target(cfg, run):
    m = run.metrics()
    assert m["final_error_m"] < 0.01
    assert m["min_error_m"] < 0.01
    assert m["time_to_1cm_s"] is not None
    assert m["time_to_1cm_s"] < 3.2


def test_document_target_38_17(cfg, run):
    target = np.array([cfg.ctrl.target_x, cfg.ctrl.target_z])
    assert np.linalg.norm(run.tip[-1] - target) < 0.01


def test_activations_within_bounds(run):
    assert np.all(run.activations >= 0.0)
    assert np.all(run.activations <= 1.0)


def test_custom_target(cfg):
    run = run_reach(cfg, target=(0.30, -0.10), seconds=2.0)
    assert np.linalg.norm(run.tip[-1] - np.array([0.30, -0.10])) < 0.02


def test_bad_target_shape(cfg):
    with pytest.raises(ValueError):
        run_reach(cfg, target=(0.3, 0.1, 0.2))
