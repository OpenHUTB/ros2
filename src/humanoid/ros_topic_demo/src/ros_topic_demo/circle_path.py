#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""圆形轨迹的纯 Python 数学部分（不依赖 ROS）。

这样即使手头没有 ROS Noetic，也能在 Windows/Linux 上先验证轨迹本身：
半径、周期、朝向切线、采样数。ROS 节点 ``trajectory_node.py`` 只负责把这里的
位姿包装成 Odometry 和 TF。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys

__all__ = ["normalize_angle", "pose_at", "sample_poses", "write_csv"]


def normalize_angle(a):
    """把角度折到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def pose_at(phase, radius=1.0):
    """phase（rad）处的平面位姿，返回 ``(x, y, yaw)``。

    轨迹是圆心在原点、半径 ``radius`` 的圆；phase=0 时位于 ``(radius, 0)``，
    朝向沿逆时针切线方向。
    """
    if radius <= 0:
        raise ValueError("radius 必须为正")
    x = radius * math.cos(phase)
    y = radius * math.sin(phase)
    yaw = normalize_angle(phase + math.pi / 2.0)
    return x, y, yaw


def sample_poses(period=10.0, rate=10.0, radius=1.0):
    """按固定频率采样一个完整周期。

    返回 dict 列表，字段：``t``、``phase``、``x``、``y``、``yaw``。
    ``period`` 是一圈秒数，``rate`` 是 Hz；``period * rate`` 应为整数，
    否则会四舍五入到最近的采样数。
    """
    if period <= 0:
        raise ValueError("period 必须为正")
    if rate <= 0:
        raise ValueError("rate 必须为正")
    if radius <= 0:
        raise ValueError("radius 必须为正")

    n = int(round(period * rate))
    if n < 2:
        raise ValueError("period * rate 至少为 2")
    dt = period / n
    omega = 2.0 * math.pi / period
    rows = []
    for i in range(n):
        t = i * dt
        phase = omega * t
        x, y, yaw = pose_at(phase, radius)
        rows.append({"t": t, "phase": phase, "x": x, "y": y, "yaw": yaw})
    return rows


def write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "phase", "x", "y", "yaw"])
        for r in rows:
            writer.writerow([r["t"], r["phase"], r["x"], r["y"], r["yaw"]])


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="圆形轨迹离线自检（不需要 ROS）")
    parser.add_argument("--radius", type=float, default=1.0,
                        help="半径，默认 1.0 m")
    parser.add_argument("--period", type=float, default=10.0,
                        help="一圈秒数，默认 10.0")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="采样频率 Hz，默认 10.0")
    parser.add_argument("--csv", default="",
                        help="可选：把采样点写到 CSV")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    try:
        rows = sample_poses(args.period, args.rate, args.radius)
    except ValueError as exc:
        parser.error(str(exc))

    if args.csv:
        write_csv(args.csv, rows)
    if not args.quiet:
        first, last = rows[0], rows[-1]
        print("samples=%d radius=%.3f period=%.3fs rate=%.3fHz"
              % (len(rows), args.radius, args.period, args.rate))
        print("first: t=%.3f x=%.3f y=%.3f yaw=%.1f deg"
              % (first["t"], first["x"], first["y"],
                 math.degrees(first["yaw"])))
        print("last:  t=%.3f x=%.3f y=%.3f yaw=%.1f deg"
              % (last["t"], last["x"], last["y"],
                 math.degrees(last["yaw"])))
        if args.csv:
            print("csv: %s" % args.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
