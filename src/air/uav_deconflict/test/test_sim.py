#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环冒烟测试：真飞结果必须与解析航迹一致。

这两项是整条链路可信度的关键：

1. 两机保持 30 m 平行间隔飞行 → 真飞最小水平间隔应接近 30 m；
2. 终点位置误差应远小于机体尺寸（否则"按航迹飞行"的前提不成立）。
"""
import pytest

from uav_deconflict.core.conflict import Separation
from uav_deconflict.core.plan import Plan
from uav_deconflict.core.sim import run_fleet

pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def two_ship():
    plans = [
        Plan("A", [(0, 0), (100, 0)], alt=15.0, speed=10.0, climb_speed=2.5),
        Plan("B", [(0, -30), (100, -30)], alt=15.0, speed=10.0, climb_speed=2.5),
    ]
    return plans, run_fleet(plans, tail=1.0)


def test_truth_matches_planned_separation(two_ship):
    plans, result = two_ship
    stats = result.min_separation(Separation())
    # 计划上两机恒为 30 m 平行间隔，真飞应保持在同一量级
    assert stats["min_h"] == pytest.approx(30.0, abs=5.0)
    assert stats["pair"] == ("A", "B")


def test_arrival_error_small(two_ship):
    _plans, result = two_ship
    for uid, err in result.final_error.items():
        assert err < 1.0, "%s 终点误差 %.2f m" % (uid, err)
    for uid, err in result.max_track_error.items():
        # 起飞瞬间垂直通道存在一阶滞后，本组参数实测约 4.5 m；
        # 属于几何控制器的真实瞬态，不是控制失效；巡航段误差在 1 m 内。
        assert err < 6.5, "%s 全程最大跟踪误差 %.2f m" % (uid, err)


def test_result_shapes(two_ship):
    plans, result = two_ship
    assert set(result.pos) == {p.uid for p in plans}
    n = len(result.times)
    assert n > 100
    for arr in result.pos.values():
        assert arr.shape == (n, 3)
    assert result.steps > 0
    assert result.wall_time > 0.0
