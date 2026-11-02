#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""waypoint_trajectory_planner 单元测试（不依赖 ROS）。"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import WaypointTrajectory  # noqa: E402


def make_traj():
    # 方形航线（NED）: 起飞点 -> 东 50 -> 北 50 -> 回起点, 高度 -10
    return WaypointTrajectory(
        [(0, 0, -10), (0, 50, -10), (50, 50, -10), (0, 0, -10)],
        cruise_speed=5.0, hover_time=2.0, sample_dt=0.5)


def test_input_validation():
    try:
        WaypointTrajectory([(0, 0, 0)])
        assert False, '应拒绝单航点'
    except ValueError:
        pass
    try:
        WaypointTrajectory([(0, 0, 0), (1, 1, 1)], cruise_speed=-1)
        assert False, '应拒绝负速度'
    except ValueError:
        pass


def test_total_length():
    traj = make_traj()
    # 50 + 50 + sqrt(50^2+50^2) ≈ 170.71
    expected = 50 + 50 + math.sqrt(50 ** 2 + 50 ** 2)
    assert abs(traj.total_length - expected) < 1e-9


def test_total_time():
    traj = make_traj()
    expected = traj.total_length / 5.0 + 2.0 * 3
    assert abs(traj.total_time - expected) < 1e-9


def test_start_and_end():
    traj = make_traj()
    assert traj.position_at(0) == (0.0, 0.0, -10.0)
    assert traj.position_at(traj.total_time + 100) == (0.0, 0.0, -10.0)


def test_hover_at_waypoint():
    traj = make_traj()
    # 到达第二个航点后应悬停
    t_arrive = traj.segment_times[0]
    p1 = traj.position_at(t_arrive + 1.0)
    assert p1 == (0.0, 50.0, -10.0)


def test_speed_consistency():
    """巡航段采样位移/时间应约等于巡航速度。"""
    traj = make_traj()
    t_mid = traj.segment_times[0] / 2.0
    dt = 0.5
    p_a = traj.position_at(t_mid)
    p_b = traj.position_at(t_mid + dt)
    dist = math.sqrt(sum((p_a[i] - p_b[i]) ** 2 for i in range(3)))
    assert abs(dist / dt - 5.0) < 0.05


def test_sample_coverage():
    traj = make_traj()
    samples = traj.sample()
    assert len(samples) > 50
    assert samples[0][1] == (0.0, 0.0, -10.0)
    assert samples[-1][1] == (0.0, 0.0, -10.0)  # 末点恰为终点
