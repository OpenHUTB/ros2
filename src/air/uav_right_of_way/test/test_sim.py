#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环冒烟测试：小规模对头场景真飞（<10 s 墙钟）。"""
import numpy as np
import pytest

from uav_right_of_way.core.detect import DetectParams
from uav_right_of_way.core.scenarios import FlightPlan
from uav_right_of_way.core.sim import run_fleet

pytest.importorskip("mujoco")

DP = DetectParams(horizon_s=8.0, cpa_threshold_m=40.0)


def mini_headon():
    """小规模对头：航程减半、巡航高度 12 m，控制冒烟时长。"""
    return [
        FlightPlan("A", (-90.0, 0.0), (90.0, 0.0), alt=12.0, speed=8.0),
        FlightPlan("B", (90.0, 0.0), (-90.0, 0.0), alt=12.0, speed=8.0),
    ]


@pytest.fixture(scope="module")
def smoke():
    plans = mini_headon()
    base = run_fleet(plans, enable_rules=False, use_mujoco=True, tail=1.0,
                     detect_params=DP)
    rules = run_fleet(plans, enable_rules=True, use_mujoco=True, tail=1.0,
                      detect_params=DP)
    return base, rules


def test_baseline_has_no_decisions_and_collides(smoke):
    base, _ = smoke
    assert base.mode == "baseline"
    assert len(base.decisions) == 0
    min_d, pair, _t = base.min_separation()
    assert pair == ("A", "B")
    assert min_d < 10.0     # 无规则时两机在中线几乎重合


def test_rules_improve_separation_and_decide(smoke):
    _base, rules = smoke
    assert rules.mode == "rules"
    assert len(rules.decisions) >= 2     # 双方各出一条决策
    assert all(d.reason for d in rules.decisions)
    min_d, _pair, _t = rules.min_separation()
    assert min_d > 5.0                   # 真飞间隔显著拉开


def test_result_shapes_and_wall_time(smoke):
    base, rules = smoke
    for res in (base, rules):
        assert set(res.pos) == {"A", "B"}
        n = len(res.times)
        assert n > 50
        for arr in res.pos.values():
            assert arr.shape == (n, 3)
        assert res.steps > 0
        assert res.wall_time > 0.0
        # 真飞到达误差：终点接近计划终点
        for plan in res.plans.values():
            err = np.linalg.norm(res.pos[plan.uid][-1][:2]
                                 - np.array(plan.end))
            assert err < 6.0, "%s 终点误差 %.2f m" % (plan.uid, err)
    assert base.wall_time + rules.wall_time < 60.0
