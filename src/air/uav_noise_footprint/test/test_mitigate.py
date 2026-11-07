#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""降噪优化 core/mitigate.py：三手段、约束与贪心择优。"""
from __future__ import annotations

import math

import pytest

from uav_noise_footprint.core.exposure import objective
from uav_noise_footprint.core.mitigate import (MitigationParams,
                                               _lateral_offset_for,
                                               laeq_table, mitigate_fleet,
                                               min_point_distance)
from uav_noise_footprint.core.scenarios import get_scenario
from uav_noise_footprint.core.track import Flight

SCHOOL = get_scenario("school")


def test_altitude_raise_reduces_objective():
    f = SCHOOL.flights[0]
    base = objective([f], SCHOOL.points)
    hi = objective([f.with_altitude(60.0)], SCHOOL.points)
    assert hi < base


def test_steeper_climb_reduces_noise_near_takeoff():
    """爬升 12° 在同一地面位置上飞得更高、更快离开 → 起降段噪声更低。"""
    from uav_noise_footprint.core.exposure import SensitivePoint
    f = SCHOOL.flights[0]
    near = [SensitivePoint("起飞点旁", 100.0, -30.0)]
    assert objective([f.with_climb_angle(12.0)], near) < \
        objective([f], near)


def test_lateral_offset_computes_signed_direction():
    """巡航段中段上方的敏感点：横移方向朝远侧，移后距离 ≥ 缓冲区。"""
    from uav_noise_footprint.core.exposure import SensitivePoint
    f = SCHOOL.flights[0]
    mid_pt = [SensitivePoint("巡航中段", 0.0, 40.0)]
    off = _lateral_offset_for(f, mid_pt, MitigationParams())
    assert off is not None and not math.isnan(off)
    # 敏感点在航向 +x 的左侧（y=+40）→ 航迹向右（负值）平移避开
    assert off < 0
    shifted = f.with_lateral_offset(off)
    dists = min_point_distance(shifted, mid_pt)
    assert dists["巡航中段"] >= MitigationParams().buffer - 1.0


def test_two_sided_points_unsolvable():
    """敏感点分布在航迹两侧 → 单次平移无解（NaN）。"""
    from uav_noise_footprint.core.exposure import SensitivePoint
    f = SCHOOL.flights[0]
    pts = [SensitivePoint("L", 800.0, 50.0), SensitivePoint("R", 800.0, -50.0)]
    off = _lateral_offset_for(f, pts, MitigationParams())
    assert off is not None and math.isnan(off)


def test_mitigation_improves_and_respects_constraint():
    res = mitigate_fleet(SCHOOL.flights, SCHOOL.points, SCHOOL.params,
                         repeat=SCHOOL.events_per_hour, t_obs=3600.0)
    assert res.actions, "至少应施加一个降噪动作"
    assert res.improvement_db > 0.5
    assert res.path_increase_ratio <= res.max_path_increase + 1e-9


def test_each_action_kind_applied_once_per_uid():
    res = mitigate_fleet(SCHOOL.flights, SCHOOL.points, SCHOOL.params,
                         repeat=SCHOOL.events_per_hour, t_obs=3600.0)
    seen = {}
    for a in res.actions:
        key = (a.uid, a.kind)
        assert key not in seen, "同一飞机同一手段只施加一次"
        seen[key] = a


def test_objective_values_consistent_with_table():
    res = mitigate_fleet(SCHOOL.flights, SCHOOL.points, SCHOOL.params,
                         repeat=SCHOOL.events_per_hour, t_obs=3600.0)
    after = [res.plans[f.uid] for f in SCHOOL.flights]
    rows = laeq_table(SCHOOL.flights, after, SCHOOL.points,
                      repeat=SCHOOL.events_per_hour, t_obs=3600.0)
    for row in rows:
        assert row["laeq_after"] < row["laeq_before"]
        assert row["delta_db"] == pytest.approx(
            row["laeq_before"] - row["laeq_after"], abs=0.15)


def test_path_constraint_blocks_greedy():
    """航程约束 0 时只剩不改航程的手段（抬升/爬升角）。"""
    params = MitigationParams(max_path_increase=0.0)
    res = mitigate_fleet(SCHOOL.flights, SCHOOL.points, params,
                         repeat=SCHOOL.events_per_hour, t_obs=3600.0)
    assert res.path_increase_ratio <= 1e-9
    for a in res.actions:
        assert a.kind in ("alt_raise", "steeper_climb")
