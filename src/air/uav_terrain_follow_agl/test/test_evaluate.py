# -*- coding: utf-8 -*-
"""指标口径：间隙、AGL 误差、低于安全间隙时长与聚合。"""
import math

import pytest

from uav_terrain_follow_agl.core.evaluate import (aggregate, clearance_series,
                                                  format_table, max_climb_rate,
                                                  min_clearance, rms_agl_error,
                                                  summary, time_below)
from uav_terrain_follow_agl.core.flyer import Sample


def _samples(rows):
    """rows: (t, x, z, vz, terrain_z, agl) 的简写，其余字段按需推出。"""
    out = []
    for t, x, z, vz, h, agl in rows:
        out.append(Sample(t=t, x=x, z=z, vx=8.0, vz=vz, terrain_z=h,
                          agl_true=agl, agl_meas=agl, clearance=z - h,
                          z_cmd=z, vz_cmd=vz))
    return out


def test_clearance_and_min():
    rows = _samples([(0.0, 0.0, 15.0, 0.0, 0.0, 15.0),
                     (0.01, 1.0, 15.0, 0.0, 6.0, 9.0),
                     (0.02, 2.0, 15.0, 0.0, 18.0, -3.0)])
    assert clearance_series(rows)[2] == pytest.approx(-3.0)
    assert min_clearance(rows) == pytest.approx(-3.0)


def test_rms_agl_error_skips_warmup():
    rows = _samples([(0.0, 0.0, 100.0, 0.0, 0.0, 100.0),      # 加速段，排除
                     (10.0, 80.0, 16.0, 0.0, 0.0, 16.0),
                     (10.01, 80.1, 14.0, 0.0, 0.0, 14.0)])
    assert rms_agl_error(rows, 15.0, warmup_s=6.0) == pytest.approx(math.sqrt(1.0))


def test_time_below_counts_samples_times_dt():
    rows = _samples([(0.0, 0.0, 15.0, 0.0, 0.0, 15.0),
                     (0.01, 1.0, 6.0, 0.0, 3.0, 3.0),
                     (0.02, 2.0, 6.0, 0.0, 3.0, 3.0),
                     (0.03, 3.0, 20.0, 0.0, 3.0, 17.0)])
    assert time_below(rows, 5.0) == pytest.approx(0.02)


def test_max_climb_rate_uses_abs_after_warmup():
    rows = _samples([(0.0, 0.0, 15.0, 9.0, 0.0, 15.0),
                     (8.0, 60.0, 15.0, -2.5, 0.0, 15.0)])
    assert max_climb_rate(rows, warmup_s=6.0) == pytest.approx(2.5)


def test_summary_reports_intrusion_and_first_crossing():
    rows = _samples([(0.0, 0.0, 15.0, 0.0, 0.0, 15.0),
                     (0.01, 10.0, 15.0, 0.0, 16.0, -1.0),
                     (0.02, 20.0, 15.0, 0.0, 20.0, -5.0)])
    s = summary(rows, agl_set_m=15.0, safety_m=5.0, warmup_s=6.0)
    assert s["min_clearance_m"] == pytest.approx(-5.0)
    assert s["intrusion_m"] == pytest.approx(5.0)
    assert s["first_intrusion_x_m"] == pytest.approx(10.0)
    assert s["crashed"] is True
    assert s["samples"] == 3


def test_summary_without_intrusion_has_none_marker():
    rows = _samples([(0.0, 0.0, 15.0, 0.0, 0.0, 15.0),
                     (8.0, 60.0, 15.0, 0.0, 0.0, 15.0)])
    s = summary(rows, agl_set_m=15.0, safety_m=5.0, warmup_s=6.0)
    assert s["first_intrusion_x_m"] is None
    assert s["crashed"] is False
    assert s["intrusion_m"] == pytest.approx(0.0)


def test_empty_summary():
    assert summary([], 15.0, 5.0) == {"samples": 0}


def test_aggregate_takes_worst_clearance():
    rows = [{"min_clearance_m": 3.0, "mean_clearance_m": 10.0,
             "rms_agl_error_m": 2.0, "max_climb_rate_mps": 3.0, "crashed": False},
            {"min_clearance_m": -1.0, "mean_clearance_m": 8.0,
             "rms_agl_error_m": 4.0, "max_climb_rate_mps": 3.2, "crashed": True}]
    agg = aggregate(rows)
    assert agg["min_clearance_m"] == pytest.approx(-1.0)
    assert agg["rms_agl_error_m"] == pytest.approx(3.0)
    assert agg["crashed_runs"] == 1
    assert agg["runs"] == 2
    assert aggregate([]) == {}


def test_format_table_renders_bool_and_nan():
    text = format_table([{"a": 1.0, "b": True}, {"a": float("nan"), "b": False}],
                        columns=("a", "b"), headers=("A", "B"))
    assert "A" in text and "是" in text and "否" in text
    assert "nan" in text
