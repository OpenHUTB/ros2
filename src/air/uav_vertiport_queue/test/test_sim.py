#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环冒烟测试：真飞的离地时刻与间隔必须与排定的一致。

两架离场按固定间隔策略排定在 T1/T2（0 s 与 60 s），真飞后实际离地
间隔应接近 60 s——证明排队策略的结论在刚体动力学下可执行。
"""
import pytest

from uav_vertiport_queue.core.sim import run_fleet
from uav_vertiport_queue.core.scenarios import LAYOUT
from uav_vertiport_queue.core.track import departure_track

pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def two_ship():
    # 用较近的离场点控制仿真窗口（~68 s），保证冒烟测试秒级完成
    exit_xy = (150.0, 0.0)
    tracks = [
        departure_track("A", LAYOUT["pads_departure"]["T1"],
                        exit_xy, t_takeoff=0.0),
        departure_track("B", LAYOUT["pads_departure"]["T2"],
                        exit_xy, t_takeoff=60.0),
    ]
    return tracks, run_fleet(tracks, tail=1.0)


def test_actual_takeoff_matches_plan(two_ship):
    tracks, result = two_ship
    events = result.op_events()
    for track in tracks:
        e = events[track.uid]
        assert e["kind"] == "departure"
        assert e["actual"] is not None
        # 实际离地时刻与排定时刻偏差 < 3 s
        assert abs(e["error"]) < 3.0, "%s 偏差 %s s" % (track.uid, e["error"])


def test_actual_interval_respects_queueing(two_ship):
    _tracks, result = two_ship
    gaps = result.interval_gaps("departure")
    assert len(gaps) == 1
    # 排定间隔 60 s，真飞后不得小于 55 s（含起飞瞬态的余量）
    assert gaps[0] >= 55.0, "实际离场间隔 %.2f s < 55 s" % gaps[0]


def test_track_error_bounded(two_ship):
    _tracks, result = two_ship
    for uid, err in result.max_track_error.items():
        # 起飞瞬间垂直通道一阶滞后属几何控制器真实瞬态（参考实现实测 ~4.5 m）
        assert err < 6.5, "%s 全程最大跟踪误差 %.2f m" % (uid, err)
    for uid, err in result.final_error.items():
        assert err < 1.0, "%s 终点误差 %.2f m" % (uid, err)
