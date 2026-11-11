# -*- coding: utf-8 -*-
"""仿真闭环测试：需要 MuJoCo + 底座，缺环境时自动跳过（不算失败）。

只跑 2~4 s 的短程用例，整套测试仍在 10 s 量级。
"""

import numpy as np
import pytest

pytest.importorskip("numpy")

try:                                    # 没有底座/MuJoCo 时整组跳过
    from uav_actuator_fault_sim._compat import load_base

    load_base()
except Exception as exc:                 # pragma: no cover - 环境缺失分支
    pytest.skip(f"底座不可用：{exc}", allow_module_level=True)

from uav_actuator_fault_sim.runner import RunConfig, run_case, run_many  # noqa: E402
from uav_actuator_fault_sim.scenario import make_case                    # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return RunConfig(duration=3.0)


def test_healthy_hover_stays_up(cfg):
    r = run_case(make_case("open_ground", "healthy", seed=0), cfg=cfg)
    assert not r.crashed
    assert r.min_altitude == pytest.approx(1.5, abs=0.05)
    assert r.health.shape[1] == 4
    assert np.allclose(r.health, 1.0)


def test_single_loss_crashes_within_one_second(cfg):
    r = run_case(make_case("open_ground", "single_loss", seed=0, rotor=1, onset=2.0),
                 cfg=cfg)
    assert r.crashed
    assert r.crash_delay is not None and r.crash_delay < 1.0
    assert r.touchdown_speed is not None and r.touchdown_speed > 1.0
    # 故障增益真值：注入后该桨为 0
    assert r.health[-1, 1] == pytest.approx(0.0)


def test_partial_degradation_keeps_flying(cfg):
    r = run_case(make_case("open_ground", "partial_degradation", seed=0,
                           rotor=0, onset=1.0, severity=0.5), cfg=cfg)
    assert not r.crashed
    assert r.horizontal_drift < 1.0


def test_records_have_consistent_shapes(cfg):
    r = run_case(make_case("open_ground", "healthy", seed=0), cfg=cfg)
    n = len(r.t)
    assert r.pos.shape == (n, 3)
    assert r.vel.shape == (n, 3)
    assert r.omega.shape == (n, 3)
    assert r.euler.shape == (n, 3)
    assert r.thrusts.shape == (n, 4)
    assert r.desired is not None and r.desired.shape == (n, 4)
    assert np.all(np.diff(r.t) > 0.0)


def test_run_many_matches_run_case(cfg):
    cases = [make_case("open_ground", "healthy", seed=0),
             make_case("open_ground", "single_loss", seed=0)]
    results = run_many(cases, cfg=cfg)
    assert len(results) == 2
    assert results[0].scenario == "healthy"
    assert results[1].scenario == "single_loss"


def test_determinism_same_seed_same_result(cfg):
    case = make_case("open_ground", "single_loss", seed=7, rotor=2, onset=1.5)
    a = run_case(case, cfg=cfg)
    b = run_case(case, cfg=cfg)
    assert np.allclose(a.pos, b.pos)
    assert a.min_altitude == pytest.approx(b.min_altitude)
