#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""排队策略单元测试：FIFO / 优先级抢占 / 固定间隔 / 冲突判定 / 指标。"""
import pytest

from uav_vertiport_queue.core.queue import (ARRIVAL, DEPARTURE, Assignment,
                                            QueueConfig, Request,
                                            assign_pads_no_wait,
                                            build_queue_payload,
                                            find_conflicts, max_queue_length,
                                            schedule, schedule_metrics)

CFG = QueueConfig()


def _dep(uid, t, priority=0):
    return Request(uid, DEPARTURE, t, priority)


def _arr(uid, t, priority=0):
    return Request(uid, ARRIVAL, t, priority)


# --------------------------------------------------------------------------- #
# FIFO 基线
# --------------------------------------------------------------------------- #
def test_fifo_respects_request_order():
    reqs = [_dep("A", 0.0), _dep("B", 20.0), _dep("C", 70.0), _dep("D", 110.0)]
    sched = schedule(reqs, CFG, "fifo")
    dep = sched.of_kind(DEPARTURE)
    assert [a.uid for a in dep] == ["A", "B", "C", "D"]
    assert [a.t_scheduled for a in dep] == sorted(a.t_scheduled
                                                  for a in dep)


def test_fifo_no_pad_conflicts():
    reqs = [_dep("A", 0.0), _dep("B", 20.0), _dep("C", 25.0),
            _arr("D", 0.0), _arr("E", 10.0)]
    sched = schedule(reqs, CFG, "fifo")
    assert find_conflicts(sched.assignments, CFG) == []


def test_fifo_delays_when_pads_exhausted():
    # 3 架同时请求离场，只有 2 个起飞位：第 3 架必须等占用时长结束
    reqs = [_dep("A", 0.0), _dep("B", 0.0), _dep("C", 0.0)]
    sched = schedule(reqs, CFG, "fifo")
    times = sorted(a.t_scheduled for a in sched.assignments)
    assert times[0] == 0.0 and times[1] == 0.0
    assert times[2] == pytest.approx(CFG.occupancy_departure)
    assert find_conflicts(sched.assignments, CFG) == []


# --------------------------------------------------------------------------- #
# 优先级抢占
# --------------------------------------------------------------------------- #
def test_priority_request_not_delayed():
    # 医疗急救（priority=10）夹在 4 架普通离场中间，priority 策略下不被延误
    reqs = [_dep("A", 0.0), _dep("B", 15.0), _dep("MED", 30.0, priority=10),
            _dep("C", 45.0)]
    sched = schedule(reqs, CFG, "priority")
    med = next(a for a in sched.assignments if a.uid == "MED")
    assert med.t_scheduled == pytest.approx(30.0)
    assert med.delay == pytest.approx(0.0)


def test_priority_pushes_normal_traffic():
    # 急救插队后，普通机流的延误总和应大于"急救也按 FIFO 排"的情形
    reqs = [_dep("A", 0.0), _dep("B", 15.0), _dep("MED", 30.0, priority=10),
            _dep("C", 45.0)]
    prio = schedule_metrics(schedule(reqs, CFG, "priority").assignments,
                            CFG, 4)
    fifo = schedule_metrics(schedule(reqs, CFG, "interval").assignments,
                            CFG, 4)
    med_fifo = min(a.delay for a in schedule(reqs, CFG, "interval").assignments
                   if a.uid == "MED")
    assert prio["delay_total"] > fifo["delay_total"]
    med_prio = min(a.delay for a in schedule(reqs, CFG, "priority").assignments
                   if a.uid == "MED")
    assert med_prio <= med_fifo + 1e-6


# --------------------------------------------------------------------------- #
# 固定间隔
# --------------------------------------------------------------------------- #
def test_interval_departure_60s():
    reqs = [_dep("A", 0.0), _dep("B", 20.0), _dep("C", 70.0)]
    dep = schedule(reqs, CFG, "interval").of_kind(DEPARTURE)
    gaps = [b.t_scheduled - a.t_scheduled
            for a, b in zip(dep[:-1], dep[1:])]
    assert min(gaps) == pytest.approx(CFG.min_interval_departure)


def test_interval_arrival_90s():
    reqs = [_arr("A", 0.0), _arr("B", 30.0), _arr("C", 60.0)]
    arr = schedule(reqs, CFG, "interval").of_kind(ARRIVAL)
    gaps = [b.t_scheduled - a.t_scheduled
            for a, b in zip(arr[:-1], arr[1:])]
    assert min(gaps) == pytest.approx(CFG.min_interval_arrival)


def test_interval_configurable():
    cfg = QueueConfig(min_interval_departure=120.0)
    reqs = [_dep("A", 0.0), _dep("B", 10.0)]
    dep = schedule(reqs, cfg, "interval").of_kind(DEPARTURE)
    assert dep[1].t_scheduled - dep[0].t_scheduled == pytest.approx(120.0)


# --------------------------------------------------------------------------- #
# 冲突判定 / 指标
# --------------------------------------------------------------------------- #
def test_conflict_detector_finds_overlap():
    a = Assignment("A", DEPARTURE, "T1", 0.0, 0.0)
    b = Assignment("B", DEPARTURE, "T1", 10.0, 10.0)
    c = Assignment("C", DEPARTURE, "T2", 0.0, 0.0)
    conflicts = find_conflicts([a, b, c], CFG)
    assert len(conflicts) == 1
    assert conflicts[0]["pad"] == "T1"
    assert conflicts[0]["overlap"] == pytest.approx(
        CFG.occupancy_departure - 10.0)


def test_no_conflict_touching_intervals():
    a = Assignment("A", ARRIVAL, "L1", 0.0, 0.0)
    b = Assignment("B", ARRIVAL, "L1", CFG.occupancy_arrival,
                   CFG.occupancy_arrival)
    assert find_conflicts([a, b], CFG) == []


def test_delay_only_positive():
    # 请求时刻晚于任何约束时延误为 0，且任何延误都不为负
    reqs = [_dep("A", 0.0), _dep("B", 500.0)]
    sched = schedule(reqs, CFG, "interval")
    assert all(a.delay >= 0.0 for a in sched.assignments)
    assert sched.assignments[1].delay == pytest.approx(0.0)


def test_max_queue_length():
    reqs = [_dep("A", 0.0), _dep("B", 0.0), _dep("C", 0.0)]
    sched = schedule(reqs, CFG, "fifo")
    assert max_queue_length(sched.assignments) == 1


def test_before_vs_after_metrics():
    reqs = [_dep("A", 0.0), _dep("B", 20.0), _arr("C", 30.0),
            _arr("D", 60.0), _dep("E", 80.0)]
    before = assign_pads_no_wait(reqs, CFG)
    after = schedule(reqs, CFG, "interval")
    mb = schedule_metrics(before.assignments, CFG, len(reqs))
    ma = schedule_metrics(after.assignments, CFG, len(reqs))
    assert mb["avg_wait"] == pytest.approx(0.0)
    assert ma["avg_wait"] >= 0.0
    # 排队前存在间隔违规，排队后为 0
    assert mb["dep_interval_violations"] > 0
    assert ma["dep_interval_violations"] == 0
    assert ma["min_dep_interval"] >= CFG.min_interval_departure - 1e-6
    assert ma["min_arr_interval"] >= CFG.min_interval_arrival - 1e-6


# --------------------------------------------------------------------------- #
# ROS 节点使用的载荷函数
# --------------------------------------------------------------------------- #
def test_queue_payload_json_shape():
    reqs = [_dep("A", 0.0), _dep("B", 20.0), _arr("C", 40.0)]
    payload = build_queue_payload(reqs, CFG, "interval", now=100.0)
    assert payload["strategy"] == "interval"
    assert payload["now"] == pytest.approx(100.0)
    assert len(payload["queue"]) == 3
    entry = payload["queue"][0]
    for key in ("uid", "kind", "pad", "t_request", "t_scheduled", "wait"):
        assert key in entry
    assert payload["summary"]["scheduled"] == 3
    # 排定时刻非降序
    ts = [e["t_scheduled"] for e in payload["queue"]]
    assert ts == sorted(ts)


def test_invalid_request_kind():
    with pytest.raises(ValueError):
        Request("X", "landing", 0.0)
