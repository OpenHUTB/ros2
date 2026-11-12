#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure Pursuit 路径跟踪离线演示（不需要 ROS / Gazebo）。

跑三种路径，打印跟踪指标并输出轨迹 CSV：

    python3 pure_pursuit_demo.py                 # 全部场景
    python3 pure_pursuit_demo.py --path circle   # 只跑圆周
    python3 pure_pursuit_demo.py --lookahead 0.8 --speed 0.5
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from ackermann_kinematics import (  # noqa: E402
    MAX_SPEED,
    MAX_STEER,
    WHEELBASE,
    WHEEL_RADIUS,
    Path,
    ackermann_angles,
    circle_path,
    lane_change_path,
    min_turn_radius,
    simulate,
    wheel_speeds,
)


def start_pose_of(path):
    """以路径起点为初始位姿，朝向沿第一段。"""
    (x0, y0), (x1, y1) = path.points[0], path.points[1]
    return (x0, y0, math.atan2(y1 - y0, x1 - x0))


def s_curve_path(length=4.8, offset=0.6, samples=160):
    """双移线：一个完整正弦周期（先左后右），首尾都在 y=0。"""
    pts = []
    for i in range(samples + 1):
        x = length * i / samples
        y = offset * math.sin(2.0 * math.pi * x / length)
        pts.append((x, y))
    return Path(pts)


def run_case(name, path, lookahead, speed, dt, out_dir, verbose=True):
    start = start_pose_of(path)
    result = simulate(path, start_pose=start, lookahead=lookahead,
                      speed=speed, dt=dt)
    if verbose:
        print("\n%-10s 路径长 %.2f m  用时 %.2f s  里程 %.2f m"
              % (name, result["path_length"], result["duration"], result["distance"]))
        if result["cross_track_rms"] is None:
            print("           横向偏差 样本不足（仿真 %.2f s，还没过 %.1f s 起步段）"
                  % (result["duration"], 1.0))
        else:
            print("           横向偏差 RMS %.4f m  最大 %.4f m  转向饱和 %.1f%%  %s"
                  % (result["cross_track_rms"], result["cross_track_max"],
                     result["saturate_ratio"] * 100.0,
                     "到达终点" if result["finished"] else "未到终点"))

    if out_dir:
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        csv_path = os.path.join(out_dir, "track_%s.csv" % name)
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["t_s", "x_m", "y_m", "theta_rad", "steer_rad"])
            for row in result["trajectory"]:
                writer.writerow(["%.3f" % row[0], "%.4f" % row[1],
                                 "%.4f" % row[2], "%.4f" % row[3],
                                 "%.4f" % row[4]])
    return result


def print_ackermann_example(delta=0.2, speed=0.4):
    left, right = ackermann_angles(delta)
    v_left, v_right = wheel_speeds(speed, delta)
    print("\n阿克曼解算示例（虚拟转角 %.2f rad、车速 %.2f m/s）" % (delta, speed))
    print("  左前轮转角 %+.3f rad   右前轮转角 %+.3f rad" % (left, right))
    print("  左后轮速度 %+.3f m/s   右后轮速度 %+.3f m/s" % (v_left, v_right))
    print("  车轮角速度 左 %+.1f rad/s  右 %+.1f rad/s"
          % (v_left / WHEEL_RADIUS, v_right / WHEEL_RADIUS))
    print("  最小转弯半径 %.3f m（轴距 %.3f m、最大转角 %.2f rad）"
          % (min_turn_radius(WHEELBASE, MAX_STEER), WHEELBASE, MAX_STEER))


def main():
    ap = argparse.ArgumentParser(description="Pure Pursuit 路径跟踪离线演示")
    ap.add_argument("--path", default="all",
                    choices=["circle", "lane", "s-curve", "all"])
    ap.add_argument("--lookahead", type=float, default=0.5, help="前视距离 (m)")
    ap.add_argument("--speed", type=float, default=0.4, help="车速 (m/s)")
    ap.add_argument("--dt", type=float, default=0.01, help="仿真步长 (s)")
    ap.add_argument("--out", default=os.path.join(HERE, "demo_out"),
                    help="CSV 输出目录，传空字符串则不写文件")
    args = ap.parse_args()

    if args.speed > MAX_SPEED:
        print("车速超过上限 %.1f m/s，已截断" % MAX_SPEED)
        args.speed = MAX_SPEED

    print("=" * 62)
    print("  阿克曼小车 Pure Pursuit 路径跟踪（离线仿真）")
    print("=" * 62)
    print("  轴距 %.3f m  轮距 0.16 m  最大转角 %.2f rad  前视 %.2f m  车速 %.2f m/s"
          % (WHEELBASE, MAX_STEER, args.lookahead, args.speed))

    cases = []
    if args.path in ("circle", "all"):
        cases.append(("circle", circle_path(radius=1.0)))
    if args.path in ("lane", "all"):
        cases.append(("lane", lane_change_path(length=4.0, offset=0.6)))
    if args.path in ("s-curve", "all"):
        cases.append(("s-curve", s_curve_path()))

    for name, path in cases:
        run_case(name, path, args.lookahead, args.speed, args.dt, args.out)

    print_ackermann_example()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
