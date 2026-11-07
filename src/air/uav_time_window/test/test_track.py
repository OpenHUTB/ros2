#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""4D 航迹生成（core.track）与流水线/场景（core.pipeline、scenarios）测试。"""
import math

import numpy as np
import pytest

from uav_time_window.core.mission import Task, Uav
from uav_time_window.core.pipeline import format_report, pick_sim_uids, run_case, to_json
from uav_time_window.core.scenarios import SCENARIOS, get_scenario
from uav_time_window.core.track import RouteTrack, build_tracks
from uav_time_window.core.window import schedule_uav

CFG = get_scenario("tight").config
UAV = Uav("UAV-1", 0.0, 0.0, speed=8.0, endurance=420.0)
TASKS = {t.tid: t for t in [Task("T-A", 80, 60, 40, 60, 15),
                            Task("T-B", 160, 60, 40, 200, 15)]}


def _track():
    sched = schedule_uav(UAV, ["T-A", "T-B"], TASKS, CFG)
    return RouteTrack(uid="UAV-1", schedule=sched, uav=UAV, tasks=TASKS,
                      alt=30.0, climb_speed=2.5), sched


def test_waypoints_start_and_tasks():
    """航路点串 = 起点 + 逐任务点。"""
    tr, _ = _track()
    assert tr.waypoints == [(0.0, 0.0), (80.0, 60.0), (160.0, 60.0)]


def test_first_arrival_matches_schedule():
    """第一点到达时刻与解析时刻表严格一致（爬升顺路完成）。"""
    tr, sched = _track()
    assert tr.planned_arrival["T-A"] == pytest.approx(sched.stops[0].arrival)


def test_position_at_start_and_end():
    """起飞在起点地面，落地在最后任务点地面。"""
    tr, _ = _track()
    p0 = tr.position_at(tr.start_time)
    assert p0[0] == 0.0 and p0[1] == 0.0 and p0[2] == pytest.approx(0.0)
    pe = tr.position_at(tr.end_time)
    assert pe[0] == pytest.approx(160.0) and pe[2] == pytest.approx(0.0)


def test_hover_segment_zero_velocity():
    """服务窗口内悬停：速度为零且压在任务点上空。"""
    tr, sched = _track()
    stop = sched.stops[0]
    t_mid = 0.5 * (stop.arrival + stop.departure)
    vel = tr.velocity_at(t_mid)
    pos = tr.position_at(t_mid)
    assert np.linalg.norm(vel) == pytest.approx(0.0, abs=1e-9)
    assert pos[0] == pytest.approx(80.0) and pos[2] == pytest.approx(30.0)


def test_position_continuous_over_segments():
    """段边界位置连续（悬停→平飞→悬停→下降）。"""
    tr, _ = _track()
    for seg, nxt in zip(tr.segments[:-1], tr.segments[1:]):
        assert np.allclose(seg.position(seg.t1), nxt.position(nxt.t0), atol=1e-9)


def test_build_tracks_skips_empty_uavs():
    """没有任务的机不生成航迹。"""
    sc = get_scenario("tight")
    uavs = {u.uid: u for u in sc.uavs}
    sched = schedule_uav(uavs["UAV-1"], ["T-A"],
                         {t.tid: t for t in sc.tasks}, sc.config)
    tracks = build_tracks({"UAV-1": sched, "UAV-2": schedule_uav(
        uavs["UAV-2"], [], {t.tid: t for t in sc.tasks}, sc.config)},
        uavs, {t.tid: t for t in sc.tasks})
    assert set(tracks) == {"UAV-1"}


def test_all_scenarios_cover_all_tasks():
    """四个场景：改进后分配覆盖全部任务、不可达任务有诊断。"""
    for name in SCENARIOS:
        case = run_case(name, sim=False)
        covered = [t for tids in case.allocation.assign.values() for t in tids]
        expect = [r["tid"] for r in case.screen if not r["infeasible"]]
        assert sorted(covered) == sorted(expect), name
        assert len(case.screen) == len(case.scenario.tasks)


def test_pipeline_report_and_json():
    """报告文本与 JSON 序列化可生成且关键字段齐全。"""
    case = run_case("tight", sim=False)
    text = format_report(case)
    assert "贪心" in text and "改进后" in text
    import json as _json
    import tempfile, os
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    try:
        out = to_json([case], path)
        with open(out, encoding="utf-8") as fh:
            data = _json.load(fh)
        assert data[0]["scenario"] == "tight"
        assert "greedy" in data[0] and "final" in data[0]
        assert "per_uav" in data[0]
    finally:
        os.remove(path)


def test_pick_sim_uids_prefers_loaded():
    """闭环抽机优先任务多的机，最多 2 架。"""
    case = run_case("urgent", sim=False)
    uids = pick_sim_uids(case.allocation)
    assert len(uids) == 2
    counts = [len(case.allocation.assign[u]) for u in uids]
    all_counts = sorted((len(v) for v in case.allocation.assign.values()),
                        reverse=True)
    assert counts == all_counts[:2]


def test_scenario_windows_calibrated():
    """场景标定：tight 有正延误，loose 零延误。"""
    tight = run_case("tight", sim=False)
    loose = run_case("loose", sim=False)
    assert tight.final_report["delay_total"] > 0
    assert loose.final_report["delay_total"] == 0.0
