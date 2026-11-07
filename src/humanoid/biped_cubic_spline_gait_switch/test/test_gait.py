#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多步态生成与平滑切换测试。"""

import numpy as np

from biped_cubic_spline_gait_switch.config_loader import (
    build_gaits, build_model, build_schedule, default_params_path, load_config)
from biped_cubic_spline_gait_switch.gait import GaitPlanner


def make_planner(mode="spline"):
    cfg = load_config(default_params_path())
    return GaitPlanner(
        build_model(cfg), build_gaits(cfg), build_schedule(cfg),
        transition_time=0.6, mode=mode, stand_foot_offset=0.10,
    )


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
    planner.update(0.0, 0.002)
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


def test_spline_switch_is_smooth():
    planner = make_planner("spline")
    dt = 0.002
    prev = None
    max_step = 0.0
    for i in range(int(3.6 / dt)):
        state = planner.update(i * dt, dt)
        if prev is not None:
            max_step = max(max_step, float(np.max(np.abs(state["q"] - prev))))
        prev = state["q"]
    assert max_step < 0.02


def test_direct_switch_has_jump():
    planner = make_planner("direct")
    dt = 0.002
    prev = None
    max_step = 0.0
    for i in range(int(3.2 / dt)):
        state = planner.update(i * dt, dt)
        if prev is not None:
            max_step = max(max_step, float(np.max(np.abs(state["q"] - prev))))
        prev = state["q"]
    assert max_step > 0.05
