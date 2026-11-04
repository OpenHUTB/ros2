#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""容量评估单元测试：请求率扫描、吞吐量、等待、饱和点。"""
import pytest

from uav_vertiport_queue.core.capacity import (DEFAULT_RATES,
                                               DEFAULT_WAIT_THRESHOLD,
                                               apron_occupancy,
                                               capacity_scan,
                                               evaluate_rate,
                                               generate_demand,
                                               saturation_point)
from uav_vertiport_queue.core.queue import QueueConfig

CFG = QueueConfig()


def test_generate_demand_deterministic():
    a = generate_demand(24)
    b = generate_demand(24)
    assert [r.t_request for r in a] == [r.t_request for r in b]
    assert len(a) == 24
    # 等间隔：3600/24 = 150 s
    assert a[1].t_request - a[0].t_request == pytest.approx(150.0)
    # 离场/进场交替
    kinds = {r.kind for r in a}
    assert kinds == {"departure", "arrival"}


def test_evaluate_rate_shape():
    row = evaluate_rate(24, CFG)
    for key in ("rate", "requested", "throughput", "backlog", "avg_wait",
                "max_wait", "max_queue_length", "pad_conflicts"):
        assert key in row
    assert row["requested"] == 24


def test_low_rate_zero_wait():
    # 12 架次/h：离场 6 架/h 间隔 600 s、进场 6 架/h 间隔 600 s，
    # 均大于 60/90 s 固定间隔，不应有任何等待
    row = evaluate_rate(12, CFG)
    assert row["avg_wait"] == pytest.approx(0.0)
    assert row["backlog"] == 0
    assert row["pad_conflicts"] == 0


def test_throughput_not_exceed_request_rate():
    for rate in (12, 36, 88):
        row = evaluate_rate(rate, CFG)
        assert row["throughput"] <= rate + 1e-6


def test_wait_grows_into_saturation():
    low = evaluate_rate(24, CFG)
    high = evaluate_rate(88, CFG)
    assert high["avg_wait"] > low["avg_wait"]
    assert high["avg_wait"] > DEFAULT_WAIT_THRESHOLD
    assert high["backlog"] > 0


def test_scan_covers_requested_range():
    scan = capacity_scan(rates=DEFAULT_RATES, cfg=CFG)
    rates = [row["rate"] for row in scan]
    assert rates == sorted(rates)
    # 题目要求的 12~36 架次/h 全部覆盖
    for r in (12, 16, 20, 24, 28, 32, 36):
        assert r in rates
    assert all(row["max_queue_length"] >= 0 for row in scan)
    # 饱和端排队长度显著增长
    by_rate = {row["rate"]: row for row in scan}
    assert by_rate[88]["max_queue_length"] > by_rate[12]["max_queue_length"]


def test_saturation_point_between():
    scan = capacity_scan(rates=(12, 24, 48, 72, 88), cfg=CFG)
    sat = saturation_point(scan)
    assert 12 <= sat["rate"] < 88
    assert sat["saturated"] is True
    assert "first_saturated_rate" in sat


def test_apron_occupancy_counts():
    ops = [dict(uid="A", kind="arrival", t=0.0),
           dict(uid="B", kind="arrival", t=10.0),
           dict(uid="C", kind="departure", t=300.0),
           dict(uid="D", kind="departure", t=400.0)]
    # 默认 flight_time=600：两架进场后 600 s 内同时在场，峰值 2
    assert apron_occupancy(ops) == 2
