#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure-algorithm tests: target path, standoff reference, intercept."""

import numpy as np
import pytest

from uav_moving_target_follow.core.follow import (FollowCfg, follow_reference,
                                                  plan_intercept)
from uav_moving_target_follow.core.target import (CASE_ROAD, CASE_S_CURVE,
                                                  CASE_TOWN, GroundTarget,
                                                  make_case)

CFG = FollowCfg()


def test_target_pos_vel_straight():
    t = CASE_ROAD()
    p, v = t.pos_vel(0.0)
    assert np.allclose(p, [20.0, 0.0]) and np.allclose(v, [6.0, 0.0])
    p, v = t.pos_vel(10.0)
    assert np.allclose(p, [80.0, 0.0])
    # 走完全程后停在终点，速度归零
    p, v = t.pos_vel(t.duration_s + 30.0)
    assert np.allclose(p, [320.0, 0.0]) and np.allclose(v, [0.0, 0.0])


def test_target_pos_vel_after_turn():
    t = CASE_TOWN()
    # 第一段 150 m / 6 m/s = 25 s 后进入第二段（沿 +y）
    p, v = t.pos_vel(25.0 + 5.0)
    assert np.allclose(p, [170.0, 30.0], atol=1e-6)
    assert np.allclose(v, [0.0, 6.0])


def test_follow_reference_behind_target():
    p_ref, v_ref, d = follow_reference((100.0, 50.0), (6.0, 0.0), CFG)
    # 目标向 +x 走：参考点在目标后方（-x），高度固定
    assert np.allclose(p_ref, [100.0 - CFG.standoff_m, 50.0, CFG.alt_m])
    assert np.allclose(v_ref, [6.0, 0.0, 0.0])
    assert np.allclose(d, [-1.0, 0.0])
    # 目标向 +y 走：参考点在 -y 侧
    p_ref, _v, d = follow_reference((100.0, 50.0), (0.0, 6.0), CFG)
    assert np.allclose(p_ref, [100.0, 50.0 - CFG.standoff_m, CFG.alt_m])
    assert np.allclose(d, [0.0, -1.0])


def test_follow_reference_holds_direction_when_stopped():
    # 目标停车：保持上一次偏移方向，而不是跳到随机一侧
    prev = np.array([0.0, -1.0])
    p_ref, v_ref, d = follow_reference((10.0, 10.0), (0.0, 0.0), CFG,
                                       prev_dir2=prev)
    assert np.allclose(d, prev)
    assert np.allclose(p_ref, [10.0, 10.0 - CFG.standoff_m, CFG.alt_m])
    assert np.allclose(v_ref, [0.0, 0.0, 0.0])
    # 没有历史方向时退回 +x
    p_ref, _v, d = follow_reference((0.0, 0.0), (0.0, 0.0), CFG)
    assert np.allclose(d, [1.0, 0.0])


def test_follow_reference_rejects_bad_cfg():
    with pytest.raises(ValueError):
        FollowCfg(standoff_m=0.0)
    with pytest.raises(ValueError):
        FollowCfg(alt_m=-1.0)
    with pytest.raises(ValueError):
        FollowCfg(cruise_speed=0.0)


def test_plan_intercept_catches_slower_target():
    t = CASE_ROAD()
    plan = plan_intercept(t, (0.0, 0.0), CFG)
    assert plan["feasible"] and plan["t_intercept_s"] is not None
    # 截获时刻的参考点距离起点不超过巡航可达距离
    t_i = plan["t_intercept_s"]
    p_ref, _v, _d = follow_reference(*t.pos_vel(t_i), CFG)
    reach = plan["climb_s"] + float(np.linalg.norm(p_ref[:2])) / CFG.cruise_speed
    assert reach <= t_i + 0.6


def test_plan_intercept_reports_infeasible_when_too_fast():
    # 目标 20 m/s > 巡航 9 m/s：追不上，给建议而不是硬算
    fast = GroundTarget("fast", ((0.0, 0.0), (2000.0, 0.0)), 20.0)
    plan = plan_intercept(fast, (0.0, 0.0), CFG)
    assert not plan["feasible"] and plan["t_intercept_s"] is None
    assert "cruise speed" in plan["advice"]


def test_make_case_and_invalid_path():
    assert make_case("town").name == "town"
    with pytest.raises(ValueError):
        GroundTarget("bad", ((0.0, 0.0),), 5.0)
    with pytest.raises(ValueError):
        GroundTarget("bad", ((0.0, 0.0), (1.0, 0.0)), 0.0)


def test_s_curve_geometry():
    t = CASE_S_CURVE()
    assert len(t.segs) == 4
    assert abs(t.total_len_m - 400.0) < 0.01
    p, v = t.pos_vel(t.duration_s)
    assert np.allclose(p, t.waypoints[-1], atol=1e-6)
