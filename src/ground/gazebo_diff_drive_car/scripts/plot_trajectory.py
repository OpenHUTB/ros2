#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_trajectory.py —— 轨迹可视化与多算法对比工具
==================================================

【任务书对应】
    「推荐使用多个场景进行测试，且包含性能评价」
    「支持多种路径规划与控制算法的对比」

【功能】
    读取 odom_recorder 输出的轨迹 CSV，绘制：
      1) XY 平面轨迹图（可叠加多条轨迹用于算法对比）
      2) 速度-时间曲线
      3) 角速度-时间曲线

【用法】
    # 单条轨迹
    python3 plot_trajectory.py trajectory_20261014_183000.csv

    # 多条轨迹对比
    python3 plot_trajectory.py laser_avoid.csv camera_avoid.csv teleop.csv

【输出】
    在当前目录生成 trajectory_plot.png 与 speed_plot.png

【依赖】
    numpy, matplotlib（Ubuntu 20.04 的 python3-matplotlib 已预装）
"""

from __future__ import print_function

import csv
import os
import sys


def load_trajectory(path):
    """读取轨迹 CSV。

    :param path: CSV 文件路径
    :return: (times, xs, ys, yaws, vs, ws) 六个列表
    """
    times, xs, ys, yaws, vs, ws = [], [], [], [], [], []

    with open(path, "r") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            try:
                times.append(float(row["timestamp"]))
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))
                yaws.append(float(row["yaw"]))
                vs.append(float(row["linear_vel"]))
                ws.append(float(row["angular_vel"]))
            except (KeyError, ValueError):
                continue

    return times, xs, ys, yaws, vs, ws


def compute_metrics(times, xs, ys, vs, ws):
    """计算性能评价指标。"""
    if not times:
        return {}

    duration = times[-1] - times[0]

    # 轨迹长度
    dist = 0.0
    for i in range(1, len(xs)):
        dist += ((xs[i] - xs[i - 1]) ** 2 + (ys[i] - ys[i - 1]) ** 2) ** 0.5

    # 直线位移
    net = ((xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2) ** 0.5

    avg_v = dist / duration if duration > 1e-6 else 0.0
    max_v = max(abs(v) for v in vs) if vs else 0.0
    avg_w = (sum(abs(w) for w in ws) / len(ws)) if ws else 0.0

    return {
        "duration": duration,
        "distance": dist,
        "net_displacement": net,
        "avg_speed": avg_v,
        "max_speed": max_v,
        "avg_angular": avg_w,
        "points": len(times),
        # 路径效率：直线位移 / 实际路程，越接近 1 说明路径越直
        "path_efficiency": net / dist if dist > 1e-6 else 0.0,
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    files = sys.argv[1:]

    try:
        import matplotlib
        matplotlib.use("Agg")  # 无显示环境，仅输出文件
        import matplotlib.pyplot as plt
    except ImportError:
        print("错误：需要 matplotlib。请执行 sudo apt install python3-matplotlib")
        sys.exit(1)

    # ------------------------------------------------------------------
    # 图 1：XY 轨迹
    # ------------------------------------------------------------------
    fig1, ax1 = plt.subplots(figsize=(9, 9))
    print("\n" + "=" * 74)
    print("                      轨迹性能评价")
    print("=" * 74)
    print("%-22s %8s %9s %9s %8s %8s" %
          ("文件", "时长(s)", "路程(m)", "位移(m)", "均速(m/s)", "效率"))
    print("-" * 74)

    for path in files:
        if not os.path.isfile(path):
            print("跳过（文件不存在）：%s" % path)
            continue

        times, xs, ys, yaws, vs, ws = load_trajectory(path)
        if not times:
            print("跳过（无数据）：%s" % path)
            continue

        label = os.path.splitext(os.path.basename(path))[0]
        m = compute_metrics(times, xs, ys, vs, ws)

        print("%-22s %8.2f %9.2f %9.2f %8.3f %8.2f" % (
            label[:22], m["duration"], m["distance"],
            m["net_displacement"], m["avg_speed"], m["path_efficiency"]
        ))

        # 绘制轨迹
        ax1.plot(xs, ys, linewidth=1.6, label=label)
        ax1.plot(xs[0], ys[0], "o", markersize=8,
                 label="%s 起点" % label)
        ax1.plot(xs[-1], ys[-1], "s", markersize=8,
                 label="%s 终点" % label)

        # ------------------------------------------------------------------
        # 图 2：速度曲线
        # ------------------------------------------------------------------
        fig2, (ax2, ax3) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        t0 = times[0]
        t = [x - t0 for x in times]
        ax2.plot(t, vs, linewidth=1.2)
        ax2.set_ylabel("linear velocity (m/s)")
        ax2.set_title("Linear / Angular Velocity over Time")
        ax2.grid(True, alpha=0.3)
        ax3.plot(t, ws, linewidth=1.2, color="orange")
        ax3.set_xlabel("time (s)")
        ax3.set_ylabel("angular velocity (rad/s)")
        ax3.grid(True, alpha=0.3)
        fig2.tight_layout()
        out2 = "speed_plot_%s.png" % label
        fig2.savefig(out2, dpi=110)
        plt.close(fig2)
        print("    速度曲线已保存：%s" % out2)

    print("=" * 74)

    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("OpenHUTB UGV Trajectory Comparison")
    ax1.grid(True, alpha=0.3)
    ax1.axis("equal")
    ax1.legend(loc="best", fontsize=8)
    fig1.tight_layout()
    fig1.savefig("trajectory_plot.png", dpi=110)
    print("轨迹对比图已保存：trajectory_plot.png\n")


if __name__ == "__main__":
    main()
