#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多步态生成与平滑切换测试（覆盖完整 34 s 调度）。"""

import numpy as np

from biped_cubic_spline_gait_switch.config_loader import (
    build_gaits, build_model, build_schedule, default_params_path, load_config)
from biped_cubic_spline_gait_switch.gait import GaitPlanner

DT = 0.002
DURATION = 34.0


def make_planner(mode="spline"):
    cfg = load_config(default_params_path())
    return GaitPlanner(
        build_model(cfg), build_gaits(cfg), build_schedule(cfg),
        transition_time=0.6, mode=mode, stand_foot_offset=0.10,
    )


def run_planner(mode):
    planner = make_planner(mode)
    states = []
    for i in range(int(round(DURATION / DT)) + 1):
        states.append(planner.update(i * DT, DT))
    return planner, states


def switch_starts(planner):
    return [start for start, _ in planner.schedule[1:]]


def state_arrays(states):
    times = np.array([s["elapsed"] for s in states])
    dq = np.stack([s["dq"] for s in states], axis=0)
    return times, dq


def window_max_jump(states, t0, t1):
    times, dq = state_arrays(states)
    idx = np.where((times >= t0) & (times <= t1))[0]
    if len(idx) < 2:
        return 0.0
    return float(np.max(np.abs(np.diff(dq[idx], axis=0))))


def test_schedule_and_velocity():
    planner = make_planner()
    assert planner.gait_at(0.0) == "stand"
    assert planner.gait_at(3.1) == "walk"
    assert planner.gait_at(9.1) == "fast_walk"
    assert planner.gait_at(15.1) == "run"
    assert planner.gait_at(21.1) == "walk"
    assert planner.gait_at(27.1) == "stand"
    assert abs(planner.gaits["walk"].velocity - 2 * 0.22 / 1.2) < 1e-9


def test_stand_has_two_contacts():
    planner = make_planner()
    planner.update(0.0, DT)
    state = planner._state("stand", 0.0)
    assert state["contacts"] == [True, True]


def test_walk_has_double_support():
    planner = make_planner()
    contacts = []
    for phase in np.linspace(0, 1, 100, endpoint=False):
        left = planner._foot_relative(planner.gaits["walk"], "left", phase)[2]
        right = planner._foot_relative(planner.gaits["walk"], "right", phase)[2]
        contacts.append(left and right)
    assert np.mean(contacts) > 0.15


def test_run_has_flight_phase():
    planner = make_planner()
    flight = []
    for phase in np.linspace(0, 1, 100, endpoint=False):
        left = planner._foot_relative(planner.gaits["run"], "left", phase)[2]
        right = planner._foot_relative(planner.gaits["run"], "right", phase)[2]
        flight.append((not left) and (not right))
    assert np.mean(flight) > 0.08


def test_swing_foot_lifts():
    planner = make_planner()
    zs = [planner._foot_relative(planner.gaits["walk"], "left", p)[1]
          for p in np.linspace(0, 1, 200, endpoint=False)]
    assert max(zs) > 0.08


def test_spline_is_smooth_over_full_schedule():
    planner, states = run_planner("spline")
    starts = switch_starts(planner)
    assert len(starts) == 5
    switches = sum(1 for s in states if s["switch_event"])
    assert switches == 5
    for t0 in starts:
        jump = window_max_jump(states, t0, t0 + 0.6)
        assert jump < 0.5, "切换 %.1fs 处相邻采样速度差 %.4f rad/s" % (t0, jump)


def test_spline_velocity_is_continuous_at_all_switches():
    planner, states = run_planner("spline")
    times, dq = state_arrays(states)
    starts = switch_starts(planner)
    assert starts == [3.0, 9.0, 15.0, 21.0, 27.0]
    # 五次切换首拍速度都应连续（左右极限相同）
    for t_switch in starts:
        idx = int(np.searchsorted(times, t_switch))
        assert np.allclose(dq[idx], dq[idx - 1], atol=1e-9), \
            "%.1f s 处速度不连续" % t_switch


def test_direct_switch_has_jump_over_full_schedule():
    planner, states = run_planner("direct")
    starts = switch_starts(planner)
    jumps = [window_max_jump(states, t0, t0 + 0.6) for t0 in starts]
    assert max(jumps) > 1.0, "直接切换未观察到相邻采样速度差：%s" % jumps


def test_walk_to_run_window_is_covered():
    planner, states = run_planner("spline")
    t_switch = next(start for start, name in planner.schedule if name == "run")
    assert t_switch == 15.0
    assert window_max_jump(states, t_switch, t_switch + 0.6) < 0.2
