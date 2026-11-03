#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线监视判定内核：``core/monitor.py``。

这些用例不依赖 ROS——节点在真实运行时只是订阅里程计、调用同一套函数、
把结果发成话题，所以判定逻辑的正确性可以在这里完整覆盖。
"""
from __future__ import annotations

import math

import pytest

from uav_deconflict.core.conflict import Separation
from uav_deconflict.core.monitor import (Closest, closest, evaluate,
                                         fresh_positions)

SEP = Separation(50.0, 20.0)


def test_single_aircraft_has_no_pair():
    """只有一架时没有"最近对"，也不该被判成侵入。"""
    r = closest({"UAV-1": (0.0, 0.0, 50.0)}, SEP)
    assert r.pair == ("", "")
    assert r.min_h is None and r.min_v is None
    assert r.aircraft == 1
    assert not r.violating


def test_empty_frame():
    r = closest({}, SEP)
    assert r.aircraft == 0
    assert r.min_h is None
    assert not r.violating


def test_picks_horizontal_closest_pair():
    """三机里挑水平最近的那一对，垂直差取同一对的高差。"""
    pos = {
        "UAV-1": (0.0, 0.0, 100.0),
        "UAV-2": (30.0, 40.0, 130.0),   # 与 UAV-1 水平 50 m、垂直 30 m
        "UAV-3": (200.0, 0.0, 100.0),   # 远
    }
    r = closest(pos, SEP)
    assert r.pair == ("UAV-1", "UAV-2")
    assert r.min_h == pytest.approx(50.0)
    assert r.min_v == pytest.approx(30.0)
    # 水平正好 50 m 不满足 "< 50"，判安全
    assert not r.violating


def test_violation_requires_both_axes():
    """圆柱间隔：水平破但垂直安全 → 不算侵入（本模块刻意不按三维距离判）。"""
    pos = {
        "UAV-1": (0.0, 0.0, 100.0),
        "UAV-2": (10.0, 0.0, 140.0),   # 水平 10 m（破），垂直 40 m（安全）
    }
    r = closest(pos, SEP)
    assert r.min_h == pytest.approx(10.0)
    assert not r.violating


def test_both_axes_broken_is_violation():
    pos = {
        "UAV-1": (0.0, 0.0, 100.0),
        "UAV-2": (10.0, 10.0, 105.0),  # 水平 14.14 m、垂直 5 m
    }
    r = closest(pos, SEP)
    assert r.violating
    assert r.min_h == pytest.approx(math.hypot(10.0, 10.0))


def test_stale_aircraft_is_dropped():
    """过期数据（掉线/未起飞）不参与判定。"""
    pos = {"UAV-1": (0.0, 0.0, 100.0), "UAV-2": (5.0, 0.0, 100.0)}
    stamps = {"UAV-1": 10.0, "UAV-2": 1.0}
    fresh = fresh_positions(pos, stamps, now=10.0, stale_timeout=1.0)
    assert list(fresh) == ["UAV-1"]
    assert closest(fresh, SEP).pair == ("", "")


def test_evaluate_combines_filter_and_judgement():
    """evaluate = 新鲜度过滤 + 最近对；过期的那架不能"制造"冲突。"""
    pos = {"UAV-1": (0.0, 0.0, 100.0), "UAV-2": (0.0, 0.0, 101.0)}
    stamps = {"UAV-1": 10.0, "UAV-2": 0.0}   # UAV-2 早已失联
    r = evaluate(pos, stamps, now=10.0, sep=SEP, stale_timeout=0.5)
    assert r.aircraft == 1
    assert not r.violating


def test_fresh_data_still_detects_conflict():
    pos = {"UAV-1": (0.0, 0.0, 100.0), "UAV-2": (1.0, 0.0, 100.0)}
    stamps = {"UAV-1": 10.0, "UAV-2": 9.9}
    assert evaluate(pos, stamps, now=10.0, sep=SEP, stale_timeout=0.5).violating


def test_status_dict_rounds_and_keeps_sep():
    """发布用的字典：两位小数、机对为列表、带间隔标准与累计违规数。"""
    r = Closest(pair=("A", "B"), min_h=12.3456, min_v=1.2345,
                violating=True, aircraft=2)
    d = r.as_dict(SEP, violations=7)
    assert d["min_h"] == pytest.approx(12.35)
    assert d["min_v"] == pytest.approx(1.23)
    assert d["pair"] == ["A", "B"]
    assert d["violations"] == 7
    assert d["sep"] == {"horizontal": 50.0, "vertical": 20.0}


def test_status_dict_handles_empty_frame():
    d = closest({}, SEP).as_dict(SEP, violations=0)
    assert d["min_h"] is None and d["min_v"] is None
    assert d["aircraft"] == 0
