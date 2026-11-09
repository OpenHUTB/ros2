#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""r2d2_cruise 的本地测试（不需要 ROS / 图形界面）.

覆盖重点
--------
1. 轮子关节名与 r2d2.urdf 一致（左右各 2 个，忘记改名也不会静默失效）
2. `main.py --direct --duration N --log csv` 能跑通并写出轨迹
3. 运动学验收：固定轮速差下确实在走圆弧（用折线弧长与解缠后的转角算半径）
4. 回归项：车体高度稳定在 0.47 m 左右（曾经因为按 0.2 m 生成导致插地弹飞）
5. 参数校验：DIRECT 模式没给 --duration 时必须拒绝运行（否则循环不会结束）
"""
from __future__ import annotations

import csv
import importlib.util
import math
import os
import subprocess
import sys
from functools import lru_cache

import pytest

pytest.importorskip("pybullet", reason="需要 pybullet（conda install -c conda-forge pybullet）")

_HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(_HERE, "..", "src", "ground", "r2d2_cruise"))
MAIN = os.path.join(PKG, "main.py")

DURATION = 5.0


@lru_cache(maxsize=1)
def _load_main():
    """按文件路径加载 main.py，避免依赖 sys.path 顺序（仓库里同名 main.py 很多）。"""
    spec = importlib.util.spec_from_file_location("r2d2_cruise_main", MAIN)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _wrap(angle):
    """把角度差归一化到 (-pi, pi]，用于跨 ±pi 的累计。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _run_headless(tmp_path):
    log = tmp_path / "run.csv"
    proc = subprocess.run(
        [sys.executable, MAIN, "--direct", "--duration", str(DURATION), "--log", str(log)],
        cwd=PKG,
        capture_output=True,
        text=True,
        timeout=300,
    )
    return proc, log


def test_run_headless_writes_trajectory(tmp_path):
    proc, log = _run_headless(tmp_path)
    assert proc.returncode == 0, proc.stderr
    assert log.exists(), "没有写出轨迹 CSV"

    with open(log, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))

    # 240 Hz 采样、10 Hz 记录：5 秒约 50 行
    assert len(rows) >= int(DURATION * 10) - 3
    header = set(rows[0].keys())
    assert header == {"t", "x", "y", "z", "yaw", "left_vel", "right_vel"}


def test_differential_drive_makes_a_circle(tmp_path):
    proc, log = _run_headless(tmp_path)
    assert proc.returncode == 0, proc.stderr

    with open(log, encoding="utf-8") as fh:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(fh)]

    # 弧长用折线累加（弦长会系统性偏短约 7%）；转角逐步解缠后再累加
    path_length = 0.0
    turn = 0.0
    for previous, current in zip(rows, rows[1:]):
        path_length += math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
        turn += _wrap(current["yaw"] - previous["yaw"])
    turn = abs(turn)

    assert path_length > 0.5, "5 秒内走过的弧长过小：%.3f m" % path_length
    assert 0.8 < turn < 2.5, "5 秒内转角异常：%.3f rad" % turn

    # 圆弧半径 = 弧长 / 转角；5 秒窗口（仍在加速）实测约 0.80 m
    radius = path_length / turn
    assert 0.4 < radius < 1.2, "转弯半径异常：%.2f m（5 秒窗口期望 ~0.80 m）" % radius

    # 轮速确实按左右差速给定
    first = rows[0]
    assert abs(first["left_vel"] - 10.0) < 1.5
    assert abs(first["right_vel"] - 2.0) < 1.5

    # 车体高度稳定在地面上（回归：曾被初始穿插弹飞）
    heights = [r["z"] for r in rows]
    assert min(heights) > 0.40 and max(heights) < 0.60, "车体高度异常：%.2f~%.2f" % (min(heights), max(heights))


def test_direct_mode_requires_duration():
    """DIRECT 没有窗口可关，--duration<=0 必须直接拒绝，不能进入死循环。"""
    module = _load_main()
    assert module.main(["--direct"]) == 2


def test_wheel_joint_names_match_urdf():
    """按关节名找轮子的逻辑必须和 urdf 实际关节名对得上。"""
    r2d2_main = _load_main()

    import pybullet as p
    import pybullet_data

    client = p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        rid = p.loadURDF("r2d2.urdf", [0, 0, 0.5])
        left, right = r2d2_main.find_wheels(rid)
        assert len(left) == 2 and len(right) == 2
        names = {p.getJointInfo(rid, j)[1].decode("utf-8") for j in left + right}
        assert names == set(r2d2_main.LEFT_JOINT_NAMES + r2d2_main.RIGHT_JOINT_NAMES)
    finally:
        p.disconnect(client)
