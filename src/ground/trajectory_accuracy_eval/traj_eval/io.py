#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹 CSV 读写。

统一格式（带表头，UTF-8，逗号分隔）::

    t,x,y,yaw
    0.0000,0.000000,0.000000,0.000000
    ...

``t`` 为时间戳（秒），``yaw`` 为朝向（弧度）。只要求 ``x,y,yaw`` 三列存在，
``t`` 缺失时按行号 * dt 补齐，方便直接从别的日志里导出后使用。
"""

from __future__ import annotations

import csv
import os
from typing import List, Sequence

from .se2 import Pose, wrap_angle


def save_trajectory(path: str, poses: Sequence[Pose],
                    times: Sequence[float] | None = None,
                    dt: float = 0.05) -> str:
    """把轨迹写成 CSV。``times`` 省略时按行号 * dt 生成。"""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    if times is None:
        times = [i * dt for i in range(len(poses))]
    if len(times) != len(poses):
        raise ValueError("times 与 poses 长度必须一致")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "x", "y", "yaw"])
        for t, p in zip(times, poses):
            writer.writerow([f"{t:.4f}", f"{p.x:.6f}", f"{p.y:.6f}", f"{p.yaw:.6f}"])
    return path


def load_trajectory(path: str, default_dt: float = 0.05) -> List[Pose]:
    """从 CSV 读轨迹。

    支持有表头（含 x/y/yaw 列名）与无表头（取前 3 或 4 列）两种格式。
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"轨迹文件不存在: {path}")

    with open(path, "r", encoding="utf-8-sig", newline="") as fh:
        rows = list(csv.reader(fh))
    if not rows:
        raise ValueError("CSV 为空")

    header = [c.strip().lower() for c in rows[0]]
    has_header = ("x" in header) and ("y" in header)
    if has_header:
        ix, iy = header.index("x"), header.index("y")
        iyaw = header.index("yaw") if "yaw" in header else None
        body = rows[1:]
    else:
        ix, iy = 0, 1
        iyaw = 2 if len(rows[0]) >= 3 else None
        body = rows

    poses: List[Pose] = []
    for r in body:
        if not r or not "".join(r).strip():
            continue
        try:
            x, y = float(r[ix]), float(r[iy])
            yaw = float(r[iyaw]) if iyaw is not None and iyaw < len(r) else 0.0
        except (ValueError, IndexError) as exc:
            raise ValueError(f"CSV 行解析失败: {r}") from exc
        poses.append(Pose(x, y, wrap_angle(yaw)))

    if len(poses) < 2:
        raise ValueError("有效位姿少于 2 个")
    return poses
