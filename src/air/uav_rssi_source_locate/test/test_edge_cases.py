#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure-algorithm tests: path loss model, trilateration, search geometry."""

import numpy as np
import pytest

from uav_rssi_source_locate.core.cases import CASES, make_scenario
from uav_rssi_source_locate.core.flyer import _in_area, _mirror_estimate
from uav_rssi_source_locate.core.locator import (SearchCfg, SearchState,
                                                 best_waypoint,
                                                 expanding_square, localize,
                                                 ring_points)
from uav_rssi_source_locate.core.rssi import RssiCfg

CFG = SearchCfg()
RC = RssiCfg()
BEACON = np.array([140.0, 90.0, 0.5])


def _sample(x, y, z=CFG.alt_m):
    p = np.array([float(x), float(y), float(z)])
    return (p, RC.rssi_at(float(np.linalg.norm(p - BEACON))))


def _ring_samples(center=(140.0, 90.0), radius=10.0, k=8, alt=CFG.alt_m):
    out = []
    for p in ring_points(center, radius, k):
        p3 = np.array([p[0], p[1], alt])
        out.append((p3, RC.rssi_at(float(np.linalg.norm(p3 - BEACON)))))
    return out


# --------------------------------------------------------------------- #
# 电波模型
# --------------------------------------------------------------------- #
def test_rssi_decreases_with_distance():
    vals = [RC.rssi_at(d) for d in (2.0, 5.0, 10.0, 20.0, 40.0)]
    assert all(np.isfinite(v) for v in vals)
    assert all(a > b for a, b in zip(vals, vals[1:]))


def test_rssi_is_quantized_to_half_db():
    for d in np.arange(1.5, 40.0, 1.7):
        r = RC.rssi_at(float(d))
        assert abs(r / RC.quant_db - round(r / RC.quant_db)) < 1e-9


def test_beyond_detect_range_is_not_a_reading():
    assert np.isnan(RC.rssi_at(RC.detect_range_m * 1.2))
    assert np.isfinite(RC.rssi_at(RC.detect_range_m * 0.95))
    # 底噪本身就是"可检测半径"的定义点
    assert RC.rssi_at(RC.detect_range_m) == pytest.approx(RC.floor)


def test_quantization_error_stays_within_three_percent():
    # 就近取整（不是向下取整）：反演距离的偏差应落在 ±3% 内，且没有
    # 系统性正偏——floor 量化会让每个距离都偏大 ~2%
    errs = []
    for d in np.arange(3.0, 40.0, 0.9):
        d_inv = RC.distance_from(RC.rssi_at(float(d)))
        errs.append((d_inv - d) / d)
    assert max(abs(e) for e in errs) < 0.03
    assert abs(float(np.mean(errs))) < 0.01


def test_bad_model_parameters_rejected():
    with pytest.raises(ValueError):
        RssiCfg(n=0.0)
    with pytest.raises(ValueError):
        RssiCfg(d0=-1.0)
    with pytest.raises(ValueError):
        RssiCfg(quant_db=-0.5)
    with pytest.raises(ValueError):
        RC.distance_from(float("nan"))
    with pytest.raises(ValueError):
        RC.rssi_at(0.0)


# --------------------------------------------------------------------- #
# 三边定位
# --------------------------------------------------------------------- #
def test_localize_recovers_beacon_from_clean_ring():
    out = localize(_ring_samples(), RC)
    assert out["feasible"]
    assert np.linalg.norm(out["estimate"] - BEACON[:2]) < 0.5
    assert out["residual_m"] < 1.0
    assert out["n_samples"] == 8


def test_localize_needs_three_samples():
    out = localize(_ring_samples()[:2], RC)
    assert not out["feasible"] and "3" in out["reason"]


def test_localize_rejects_collinear_samples():
    sam = [_sample(x, 90.0) for x in (110.0, 120.0, 130.0, 140.0)]
    out = localize(sam, RC)
    assert not out["feasible"] and out["reason"] == "collinear samples"


def test_localize_uses_the_3d_altitude_of_each_sample():
    # 同一批 RSSI，把样本压到地面高度：20 m 的高度差被当成 0，
    # 反演距离变成水平距离 → 解整体外移（实测 20.6 m），必须保留 3D
    sam = _ring_samples()
    flat = [(np.array([p[0], p[1], 0.5]), r) for p, r in sam]
    good = localize(sam, RC)
    bad = localize(flat, RC)
    assert good["feasible"] and bad["feasible"]
    assert np.linalg.norm(good["estimate"] - BEACON[:2]) < 0.5
    assert np.linalg.norm(bad["estimate"] - BEACON[:2]) > 10.0


def test_localize_resists_a_bad_initial_guess():
    # 单起点会陷局部极小（同一组样本残差 6.3 m vs 0.4 m）；多起点
    # 应该无论初值多离谱都收敛到同一个解
    sam = _ring_samples()
    a = localize(sam, RC, init=np.array([40.0, 20.0]))
    b = localize(sam, RC, init=np.array([200.0, 150.0]))
    assert np.linalg.norm(a["estimate"] - b["estimate"]) < 1.0
    assert np.linalg.norm(a["estimate"] - BEACON[:2]) < 0.5


def test_best_waypoint_baseline_is_grid_resolution_limited():
    # 朴素基线：只挑 RSSI 最强的航点，精度被 30 m 步长锁死
    wpts = [np.asarray(w, dtype=float) for w in expanding_square(CFG)]
    est, peak = best_waypoint(wpts, lambda x, y: RC.rssi_at(float(
        np.linalg.norm(np.array([x, y, CFG.alt_m]) - BEACON))))
    assert any(np.allclose(est, w) for w in wpts)
    assert np.linalg.norm(est - BEACON[:2]) > 10.0
    assert peak <= RC.rssi0


# --------------------------------------------------------------------- #
# 搜索几何与镜像
# --------------------------------------------------------------------- #
def test_expanding_square_starts_centered_and_grows():
    wpts = expanding_square(CFG)
    assert np.allclose(wpts[0], (CFG.area[0] / 2.0, CFG.area[1] / 2.0))
    # 每四个点一个环，环的半径单调增
    rings = [wpts[1 + 4 * i: 5 + 4 * i] for i in range((len(wpts) - 1) // 4)]
    half = [abs(r[0][0] - CFG.area[0] / 2.0) for r in rings]
    assert all(b > a for a, b in zip(half, half[1:]))
    with pytest.raises(ValueError):
        SearchCfg(ring_points=2)


def test_ring_points_are_on_the_circle():
    pts = ring_points((10.0, 20.0), 6.0, 8)
    assert len(pts) == 8
    assert all(abs(np.linalg.norm(p - [10.0, 20.0]) - 6.0) < 1e-9
               for p in pts)


def test_mirror_estimate_flips_across_the_sample_axis():
    state = SearchState(cfg=CFG)
    # 样本沿 x 轴排开（单侧弧的近似共线情形），主轴 = x 轴
    state.samples = [_sample(x, 90.0) for x in (100.0, 110.0, 120.0, 130.0)]
    state.estimate = np.array([150.0, 140.0])
    flipped = _mirror_estimate(state)
    assert np.allclose(flipped, [150.0, 40.0], atol=1e-9)
    # 翻转是自逆的：再翻一次回到原估计
    state.estimate = flipped
    assert np.allclose(_mirror_estimate(state), [150.0, 140.0], atol=1e-9)
    # 估计点本来就在主轴上：镜像不动（这种情况下只能靠出界/无信号判据）
    state.estimate = np.array([150.0, 90.0])
    assert np.allclose(_mirror_estimate(state), [150.0, 90.0], atol=1e-9)


def test_in_area_gate():
    assert _in_area([10.0, 10.0], CFG)
    assert _in_area([CFG.area[0], CFG.area[1]], CFG)
    assert not _in_area([-5.0, 10.0], CFG)
    assert not _in_area([10.0, CFG.area[1] + 1.0], CFG)


def test_every_scenario_puts_the_beacon_inside_the_area():
    for name in CASES:
        sc = make_scenario(name)
        assert _in_area(sc.beacon_xy, sc.cfg), name
    with pytest.raises(KeyError):
        make_scenario("nowhere")
