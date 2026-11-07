#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""时间窗约束检验（core.window）单元测试。"""
import math

import pytest

from uav_time_window.core.mission import Config, Task, Uav
from uav_time_window.core.window import (EARLY, LATE, ON_TIME, fleet_report,
                                         objective, schedule_uav, uav_cost)

CFG = Config()
TASKS = {
    "T1": Task("T1", 80.0, 0.0, 20.0, 60.0, service_time=15.0),
    "T2": Task("T2", 80.0, 60.0, 40.0, 80.0, service_time=15.0),
    "T3": Task("T3", 0.0, 60.0, 0.0, 30.0, service_time=15.0),
}
UAV = Uav("UAV-1", 0.0, 0.0, speed=8.0, endurance=420.0)


def test_arrival_first_leg():
    """到达时刻 = 起飞 + 欧氏距离/速度。"""
    sched = schedule_uav(UAV, ["T1"], TASKS, CFG)
    expect = math.dist(UAV.pos, TASKS["T1"].pos) / UAV.speed
    assert sched.stops[0].arrival == pytest.approx(expect)


def test_arrival_chain_includes_service():
    """第二点到达 = 第一点离开 + 航段飞行时间（服务时长计入）。"""
    sched = schedule_uav(UAV, ["T1", "T2"], TASKS, CFG)
    s0, s1 = sched.stops
    expect = (s0.departure
              + math.dist(TASKS["T1"].pos, TASKS["T2"].pos) / UAV.speed)
    assert s1.arrival == pytest.approx(expect)


def test_early_wait_and_hover_cost():
    """早到需悬停等待到 earliest，等待折算进单机成本。"""
    task = Task("X", 10.0, 0.0, 100.0, 120.0, service_time=15.0)
    sched = schedule_uav(UAV, ["X"], {"X": task}, CFG)
    stop = sched.stops[0]
    assert stop.status == EARLY
    assert stop.wait == pytest.approx(100.0 - stop.arrival)
    assert stop.serve_start == pytest.approx(100.0)
    assert uav_cost(sched, CFG) == pytest.approx(
        CFG.hover_weight * stop.wait)


def test_late_lateness_positive():
    """晚到计正延误 = 服务开始 − latest。"""
    task = Task("Y", 1000.0, 0.0, 0.0, 50.0, service_time=15.0)
    sched = schedule_uav(UAV, ["Y"], {"Y": task}, CFG)
    stop = sched.stops[0]
    assert stop.status == LATE
    assert stop.lateness == pytest.approx(stop.arrival - 50.0)
    assert stop.lateness > 0


def test_on_time_no_wait_no_lateness():
    """窗口内到达：on-time，无等待无延误。"""
    task = Task("Z", 240.0, 0.0, 10.0, 60.0, service_time=15.0)
    sched = schedule_uav(UAV, ["Z"], {"Z": task}, CFG)
    stop = sched.stops[0]
    assert stop.status == ON_TIME
    assert stop.wait == 0.0 and stop.lateness == 0.0


def test_objective_includes_fairness_spread():
    """目标函数 = Σ正延误 + λ×(最大单机延误 − 最小单机延误)。"""
    from uav_time_window.core.window import UavSchedule, StopRecord
    cfg = Config(lambda_balance=0.5)
    a = UavSchedule("A", [StopRecord("t1", 0, 0, 0, 15, (0, 10), LATE, 20.0)])
    b = UavSchedule("B", [StopRecord("t2", 0, 0, 0, 15, (0, 10), LATE, 4.0)])
    assert objective({"A": a, "B": b}, cfg) == pytest.approx(
        24.0 + 0.5 * (20.0 - 4.0))


def test_fleet_report_counts_status():
    """fleet_report 统计 early/on-time/late 计数与延误指标。"""
    sched = schedule_uav(UAV, ["T1", "T2"], TASKS, CFG)
    rep = fleet_report({"UAV-1": sched}, CFG)
    assert rep["n_early"] + rep["n_on_time"] + rep["n_late"] == 2
    assert rep["delay_total"] == pytest.approx(sched.lateness_total)
    assert rep["delay_max_uav"] == pytest.approx(sched.lateness_total)
