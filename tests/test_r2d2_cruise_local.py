#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""r2d2_cruise 的本地测试（不需要 ROS / 图形界面）.

覆盖重点
--------
1. 轮子关节名与 r2d2.urdf 一致（左右各 2 个，忘记改名也不会静默失效）
2. `main.py --direct --duration N --log csv` 能跑通并写出轨迹
3. 运动学验收：固定轮速差下确实在走圆弧（有位移、有偏航角、半径在合理范围）
4. 回归项：车体高度稳定在 0.47 m 左右（曾经因为按 0.2 m 生成导致插地弹飞）
"""
from __future__ import annotations

import csv
import os
import subprocess
import sys

import pytest

pytest.importorskip("pybullet", reason="需要 pybullet（conda install -c conda-forge pybullet）")

_HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(_HERE, "..", "src", "ground", "r2d2_cruise"))
MAIN = os.path.join(PKG, "main.py")

DURATION = 5.0


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

    first, last = rows[0], rows[-1]
    dist = ((last["x"] - first["x"]) ** 2 + (last["y"] - first["y"]) ** 2) ** 0.5
    dyaw = abs(last["yaw"] - first["yaw"])

    # 有前进位移，也有转向（不是原地打转，也不是走直线）
    assert dist > 0.3, "5 秒内位移过小：%.3f m" % dist
    assert 0.8 < dyaw < 2.5, "5 秒内转角异常：%.3f rad" % dyaw

    # 圆弧半径 = 弧长 / 转角，实测约 0.67 m
    radius = dist / dyaw
    assert 0.4 < radius < 1.2, "转弯半径异常：%.2f m（期望 ~0.67 m）" % radius

    # 轮速确实按左右差速给定
    assert abs(first["left_vel"] - 10.0) < 1.5
    assert abs(first["right_vel"] - 2.0) < 1.5

    # 车体高度稳定在地面上（回归：曾被初始穿插弹飞）
    heights = [r["z"] for r in rows]
    assert min(heights) > 0.40 and max(heights) < 0.60, "车体高度异常：%.2f~%.2f" % (min(heights), max(heights))


def test_wheel_joint_names_match_urdf():
    """按关节名找轮子的逻辑必须和 urdf 实际关节名对得上。"""
    sys.path.insert(0, PKG)
    try:
        import main as r2d2_main
    finally:
        sys.path.pop(0)

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
