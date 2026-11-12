#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mujoco_car_follow 无头回归测试。

不打开 MuJoCo 窗口，跑一段固定时长的闭环，检查：
1. CSV 正常写出且采样数足够；
2. 稳态纵向车心距接近 0.5 m；
3. 横向残差在阈值内。

运行：
    python3 -m pytest test/test_headless.py -q
"""

import csv
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "scripts" / "main.py"
DURATION_S = 20.0
WARMUP_SAMPLES = 6          # 0.5 s 采样，去掉前 3 s
TARGET = 0.5
MAX_MEAN_ERROR = 0.03
MAX_DISTANCE_ERROR = 0.08
MAX_LATERAL_ERROR = 0.06


@pytest.fixture(scope="module")
def csv_path(tmp_path_factory):
    out = tmp_path_factory.mktemp("car_follow") / "result.csv"
    env = dict(os.environ)
    env["MUJOCO_GL"] = "osmesa"
    proc = subprocess.run(
        [sys.executable, str(MAIN), "--headless",
         "--duration", str(DURATION_S), "--out", str(out)],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=120,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert out.exists(), proc.stdout + proc.stderr
    return out


def test_csv_has_enough_samples(csv_path):
    with open(csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) > WARMUP_SAMPLES + 5
    assert {"time_s", "distance_m", "lateral_m"} <= set(rows[0])


def test_steady_state_distance_and_lateral_error(csv_path):
    with open(csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))[WARMUP_SAMPLES:]
    distances = [float(row["distance_m"]) for row in rows]
    laterals = [float(row["lateral_m"]) for row in rows]

    mean_distance = sum(distances) / len(distances)
    max_distance_error = max(abs(d - TARGET) for d in distances)
    max_lateral_error = max(abs(y) for y in laterals)

    assert abs(mean_distance - TARGET) < MAX_MEAN_ERROR, mean_distance
    assert max_distance_error < MAX_DISTANCE_ERROR, max_distance_error
    assert max_lateral_error < MAX_LATERAL_ERROR, max_lateral_error
