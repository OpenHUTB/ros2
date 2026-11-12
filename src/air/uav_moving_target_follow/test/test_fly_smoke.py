#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Closed-loop smoke tests: real MuJoCo flight following each case."""

import numpy as np
import pytest

from uav_moving_target_follow.core.follow import (FollowCfg, follow_reference,
                                                  plan_intercept, slew_dir)
from uav_moving_target_follow.core.flyer import (follow_target,
                                                 naive_above_target)
from uav_moving_target_follow.core.target import CASE_ROAD, CASE_TOWN

CFG = FollowCfg()

pytest.importorskip("mujoco", reason="closed-loop tests need mujoco")


def test_follow_road_intercepts_and_holds_standoff():
    case = CASE_ROAD()
    res = follow_target(case, CFG)
    assert res.intercept_s is not None, "never captured the target"
    assert res.intercept_s < case.duration_s
    # 直线段没有转角：稳态跟随误差应在分米~米级
    assert res.mean_err_m < 1.0
    assert res.max_err_m < 6.0
    # 停车收尾：悬停在 standoff 参考点上
    assert res.final_err_m < 1.5
    # 物理时钟与报告一致（每循环推进一个控制周期）
    assert abs(res.sim_s - len(res.follow_err) * 0.05) < res.sim_s * 0.6


def test_follow_town_turns_stay_bounded():
    case = CASE_TOWN()
    res = follow_target(case, CFG)
    assert res.intercept_s is not None
    # 90° 转弯：偏移方向限速绕行后，转角误差应在 6 m 内
    assert res.turn_err_m < 6.0
    # 同一条真实轨迹下，"目标正上方"朴素参考的转角误差显著更大
    naive = naive_above_target(case, CFG, res)
    assert naive > 2.0 * res.turn_err_m
    assert res.mean_err_m < 1.0


def test_reference_is_always_on_standoff_ring():
    # 沿全程按控制周期递推偏移方向：任何时刻参考点到目标距离恒为
    # standoff_m，高度恒为 alt_m（限速只改方向，不改半径）
    case = CASE_TOWN()
    prev = None
    worst = 0.0
    for t in np.arange(0.0, case.duration_s, 0.01):
        p_ref, _v, d = follow_reference(*case.pos_vel(float(t)), CFG,
                                        prev, 0.01)
        p_t, _vt = case.pos_vel(float(t))
        r = float(np.linalg.norm(p_ref[:2] - p_t))
        worst = max(worst, abs(r - CFG.standoff_m))
        assert abs(p_ref[2] - CFG.alt_m) < 1e-9
        prev = d
    assert worst < 1e-9


def test_slew_dir_limits_rotation():
    # 90° 跳变：一步只转 rate·dt（10 步 × 15°/s × 0.01 s = 1.5°）
    a = np.array([-1.0, 0.0])
    b = np.array([0.0, -1.0])
    for _ in range(10):
        a = slew_dir(a, b, 0.01, rate_deg=15.0)
        assert abs(float(np.linalg.norm(a)) - 1.0) < 1e-9
    ang = np.rad2deg(np.arctan2(a[0] * b[1] - a[1] * b[0],
                                float(np.dot(a, b))))
    # 起始夹角 90°，转了 1.5°，还剩约 88.5°
    assert 88.0 < ang < 89.0


def test_intercept_plan_and_closed_loop_agree():
    case = CASE_TOWN()
    plan = plan_intercept(case, (0.0, 0.0), CFG)
    assert plan["feasible"]
    res = follow_target(case, CFG)
    # 闭环实际截获时刻应接近解析预测（差 15 s 内，含爬升与转弯影响）
    assert abs(res.intercept_s - plan["t_intercept_s"]) < 15.0
