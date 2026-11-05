#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Deposition assessment: Gaussian kernel, grid masking, metrics."""
import numpy as np
import pytest

from uav_agri_spray.core.checker import (build_grid, accumulate_deposit,
                                         evaluate_deposition)
from uav_agri_spray.core.planner import SprayConfig, plan_field
from uav_agri_spray.core.scenarios import CASES, case_outline, get_case

L_OUTLINE = np.array([[0.0, 0.0], [50.0, 0.0], [50.0, 16.0],
                      [24.0, 16.0], [24.0, 40.0], [0.0, 40.0]])


def _rect(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def test_single_line_peak_and_decay():
    grid = build_grid(_rect(0, 0, 20, 20), 1.0)
    # 扫描线放在 y=9.5，与栅格中心行 y=9.5 对齐
    dep = accumulate_deposit(grid, [(np.array([0.0, 9.5]),
                                     np.array([20.0, 9.5]))], 0.75)
    assert dep[9, 10] == pytest.approx(1.0, abs=0.02)      # 线上峰值
    # 相距 1 m（2σ/1.5）处 ≈ exp(-1/(2σ²))
    expect = float(np.exp(-1.0 / (2 * 0.75 ** 2)))
    assert dep[10, 10] == pytest.approx(expect, rel=0.05)
    assert dep[12, 10] < dep[10, 10] < dep[9, 10]           # 横向单调衰减


def test_grid_masks_outside_lshape():
    grid = build_grid(L_OUTLINE, 1.0)
    ys, xs = np.where(~grid.inside)
    assert len(ys) > 0
    # 缺口区 (x>24, y>16) 内任取点必须在田块外
    assert not grid.inside[36, 40]  # y=36.5, x=40.5
    assert grid.inside[10, 30]      # 南臂内部


def test_coverage_full_at_20pct():
    case = get_case("case_field_rect")
    cfg = SprayConfig(overlap=0.2)
    path = plan_field(case.subregions, (), cfg)
    grid = build_grid(case_outline(case), 1.0)
    dep = accumulate_deposit(grid, path.deposit_sources(), cfg.sigma)
    met = evaluate_deposition(dep, grid.inside)
    assert met.coverage_ratio >= 0.99
    assert met.missed_area_m2 == pytest.approx(0.0, abs=5.0)


def test_coverage_drops_at_10pct():
    case = get_case("case_field_rect")
    cfg = SprayConfig(overlap=0.1)
    path = plan_field(case.subregions, (), cfg)
    grid = build_grid(case_outline(case), 1.0)
    dep = accumulate_deposit(grid, path.deposit_sources(), cfg.sigma)
    met = evaluate_deposition(dep, grid.inside)
    assert met.coverage_ratio < 0.90
    assert met.missed_area_m2 > 100.0


def test_missed_area_consistent_with_coverage():
    case = get_case("case_field_rect")
    cfg = SprayConfig(overlap=0.1)
    path = plan_field(case.subregions, (), cfg)
    grid = build_grid(case_outline(case), 1.0)
    dep = accumulate_deposit(grid, path.deposit_sources(), cfg.sigma)
    met = evaluate_deposition(dep, grid.inside)
    expect = met.field_area_m2 * (1.0 - met.coverage_ratio)
    assert met.missed_area_m2 == pytest.approx(expect, abs=2.0)


def test_obstacle_case_shows_missed_zone():
    case = get_case("case_field_obstacle")
    cfg = SprayConfig(overlap=0.2)
    path = plan_field(case.subregions, case.obstacles, cfg)
    grid = build_grid(case_outline(case), 1.0)
    dep = accumulate_deposit(grid, path.deposit_sources(), cfg.sigma)
    met = evaluate_deposition(dep, grid.inside)
    # 障碍周边绕行区属禁喷区，覆盖率必须低于同等参数的矩形田
    assert met.coverage_ratio < 0.98
    assert met.missed_area_m2 > 50.0


def test_metrics_threshold_recorded():
    case = get_case("case_field_rect")
    cfg = SprayConfig(overlap=0.2)
    path = plan_field(case.subregions, (), cfg)
    grid = build_grid(case_outline(case), 1.0)
    dep = accumulate_deposit(grid, path.deposit_sources(), cfg.sigma)
    met = evaluate_deposition(dep, grid.inside)
    assert met.threshold == 0.5
    assert met.field_area_m2 == pytest.approx(1800.0, abs=2.0)
