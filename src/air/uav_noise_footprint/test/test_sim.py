#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环冒烟测试：真飞可控、真实数据可回灌噪声模型。

小规模场景（400 m 航段、25 m 巡航），实测墙钟 < 2 s。
"""
import math

import numpy as np
import pytest

from uav_noise_footprint.core.exposure import (SensitivePoint, laeq_at_points,
                                               laeq_from_log)
from uav_noise_footprint.core.sim import run_flight
from uav_noise_footprint.core.track import Flight

pytest.importorskip("mujoco")

FLIGHT = Flight("T", [(0, 0), (400, 0)], alt=25.0, speed=10.0,
                climb_angle=12.0)


@pytest.fixture(scope="module")
def result():
    return run_flight(FLIGHT, tail=1.0)


def test_tracking_error_under_limit(result):
    """爬升角改到 12° 后仍可控：全程最大跟踪误差 < 8 m（验收红线）。"""
    assert result.max_track_error < 8.0, \
        "跟踪误差 %.2f m 超限" % result.max_track_error
    assert result.final_error < 1.0


def test_result_shapes(result):
    n = len(result.times)
    assert n > 100
    assert result.pos.shape == (n, 3)
    assert result.steps > 0
    assert result.wall_time > 0.0
    # 真实高度峰值应接近巡航高度（不是纸面值）
    assert result.pos[:, 2].max() == pytest.approx(25.0, abs=3.0)


def test_real_trajectory_feeds_noise_model(result):
    """真实位置日志回灌声学模型：与计划 LAeq 偏差 < 1 dB。"""
    pts = [SensitivePoint("p", 200.0, 30.0)]
    plan = laeq_at_points([FLIGHT], pts, 2.0)["p"]
    mask = ((result.times >= FLIGHT.start_time) &
            (result.times <= FLIGHT.end_time))
    real = laeq_from_log(result.times[mask], result.pos[mask], (200.0, 30.0))
    assert abs(real - plan) < 1.0, "计划 %.2f vs 真飞 %.2f dB" % (plan, real)
