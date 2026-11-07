#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环冒烟测试：单机两任务真飞，实际到达与计划一致。

小规模（1 架机、2 个任务点、约 55 s 仿真），墙钟 < 10 s。
"""
import math

import pytest

from uav_time_window.core.mission import Config, Task, Uav
from uav_time_window.core.sim import run_fleet
from uav_time_window.core.track import RouteTrack
from uav_time_window.core.window import schedule_uav

pytest.importorskip("mujoco")

CFG = Config()
UAV = Uav("UAV-1", 0.0, 0.0, speed=8.0, endurance=420.0)
TASKS = {t.tid: t for t in [Task("S-1", 80.0, 0.0, 5.0, 90.0, 10.0),
                            Task("S-2", 80.0, 80.0, 5.0, 200.0, 10.0)]}


@pytest.fixture(scope="module")
def single_ship():
    sched = schedule_uav(UAV, ["S-1", "S-2"], TASKS, CFG)
    track = RouteTrack(uid="UAV-1", schedule=sched, uav=UAV, tasks=TASKS,
                       alt=25.0, climb_speed=2.5)
    result = run_fleet([track], tail=2.0)
    return track, sched, result


def test_arrival_close_to_planned(single_ship):
    """实际到达时刻应接近计划到达（闭环跟踪 + 检测半径内）。"""
    track, sched, result = single_ship
    for rec in result.arrivals:
        assert rec.actual is not None, "%s 未检出实际到达" % rec.tid
        assert abs(rec.deviation) < 6.0, \
            "%s 实际−计划到达偏差 %.2f s" % (rec.tid, rec.deviation)


def test_arrival_within_window(single_ship):
    """实际到达应落在时间窗内（本冒烟场景窗口刻意留足裕度）。"""
    _track, _sched, result = single_ship
    for rec in result.arrivals:
        task = TASKS[rec.tid]
        assert task.earliest <= rec.actual <= task.latest + 1e-6, \
            "%s 实际到达 %.2f s 超出窗口 %s" % (rec.tid, rec.actual, rec.window)


def test_track_error_bounded(single_ship):
    """全程最大跟踪误差有界（分段航迹 + 低通前馈下 < 6.5 m）。"""
    _track, _sched, result = single_ship
    for uid, err in result.max_track_error.items():
        assert err < 6.5, "%s 全程最大跟踪误差 %.2f m" % (uid, err)


def test_result_shapes(single_ship):
    """输出形状与计时字段齐全。"""
    _track, _sched, result = single_ship
    assert set(result.pos) == {"UAV-1"}
    n = len(result.times)
    assert n > 100
    assert result.pos["UAV-1"].shape == (n, 3)
    assert result.steps > 0 and result.wall_time > 0.0
    assert len(result.arrivals) == 2
