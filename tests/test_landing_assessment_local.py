#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""landing_assessment 新增能力的本地测试（不需要仿真数据 / ROS）.

覆盖
----
1. split conformal 的次序统计量口径（含小样本修正）
2. 区间包含点估计、下界不为负、未标定时退化为点估计
3. 风险分级只看区间上界，且阈值可覆盖
4. 训练产物 meta.json 里确实带上了标定结果与分位数阈值
"""
from __future__ import annotations

import json
import os
import sys

import pytest

pytest.importorskip("torch", reason="需要 PyTorch CPU 版")
pytest.importorskip("mujoco", reason="landing_assessment 的 data/simulation 依赖 mujoco")

_HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(_HERE, "..", "src", "air", "landing_assessment"))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from landing_assessment.inference import risk_level  # noqa: E402
from landing_assessment.learning import ALPHA, conformal_quantile, interval  # noqa: E402


def test_quantile_uses_finite_sample_order_statistic():
    residuals = [0.5, 1.0, 2.0, 3.0, 8.0]

    q, order, n = conformal_quantile(residuals, alpha=0.1)
    assert (order, n) == (5, 5)
    assert q == pytest.approx(8.0)

    # alpha=0.5 时取 ceil((n+1)*0.5)=3 小的残差
    q50, order50, _ = conformal_quantile(residuals, alpha=0.5)
    assert order50 == 3
    assert q50 == pytest.approx(2.0)


def test_interval_contains_point_and_never_goes_negative():
    meta = {"conformal": {"q": 2.0}}

    low, high = interval(meta, 1.0)
    assert low == 0.0, "下界应被截到 0，载荷不会是负数"
    assert high == pytest.approx(3.0)

    low, high = interval(meta, 5.0)
    assert low == pytest.approx(3.0)
    assert high == pytest.approx(7.0)

    assert interval({}, 4.0) == (4.0, 4.0), "未标定的旧模型退化为点估计"


def test_risk_level_uses_upper_bound_and_accepts_overrides():
    assert risk_level(10.0, elevated=17.0, high_limit=20.0) == "low"
    assert risk_level(18.0, elevated=17.0, high_limit=20.0) == "elevated"
    assert risk_level(21.0, elevated=17.0, high_limit=20.0) == "high"
    assert risk_level(21.0) == "low", "没有阈值时不做分级"


def test_trained_metadata_carries_calibration():
    meta_path = os.path.join(PKG, "models", "trained", "mlp_42", "meta.json")
    if not os.path.exists(meta_path):
        pytest.skip("本地还没有训练产物，先运行 main.py train")

    with open(meta_path, encoding="utf-8") as fh:
        meta = json.load(fh)

    conformal = meta["conformal"]
    assert conformal["coverage_target"] == pytest.approx(1 - ALPHA)
    assert conformal["calibration_episodes"] > 0
    assert conformal["q"] > 0

    thresholds = meta["risk_thresholds"]
    assert thresholds["high"] > thresholds["elevated"] > 0
