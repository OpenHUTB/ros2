#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_trajectory.py —— 轨迹数据分析与多算法对比绘图
========================================================

【任务书对应】
    「推荐使用多个场景进行测试，且包含性能评价」
    「支持多种路径规划与控制算法的对比」

【输入格式】
    兼容两种 CSV 表头：
      (A) rostopic echo -p 导出：
            %time,field.x,field.y,field.z
      (B) odom_recorder.py 导出：
            timestamp,x,y,yaw,linear_vel,angular_vel

【输出】
    - trajectory_plot.png      XY 轨迹对比图
    - metrics_report.txt       性能指标文本报告
    - speed_profile.png        速度曲线对比

【用法】
    python3 analyze_trajectory.py teleop.csv laser.csv camera.csv
"""

from __future__ import print_function

import csv
import math
import os
import sys


def load_csv(path):
    """读取轨迹 CSV，自动适配两种表头格式。

    :return: (times, xs, ys)
    """
    times, xs, ys = [], [], []

    with open(path, "r") as fp:
        reader = csv.reader(fp)
        try:
            header = next(reader)
        except StopIteration:
            return times, xs, ys

        # 归一化表头
        header = [h.strip().lower().lstrip("%") for h in header]

        def find_col(*candidates):
            """在表头中查找列索引。"""
            for cand in candidates:
                for i, h in enumerate(header):
                    if h == cand:
                        return i
            # 退化为子串匹配
            for cand in candidates:
                for i, h in enumerate(header):
                    if cand in h:
                        return i
            return None

        i_t = find_col("time", "timestamp")
        i_x = find_col("field.x", "x")
        i_y = find_col("field.y", "y")

        if i_t is None or i_x is None or i_y is None:
            print("  警告：无法识别表头 %s" % header)
            return times, xs, ys

        for row in reader:
            if len(row) <= max(i_t, i_x, i_y):
                continue
            try:
                t = float(row[i_t])
                x = float(row[i_x])
                y = float(row[i_y])
            except ValueError:
                continue
            times.append(t)
            xs.append(x)
            ys.append(y)

    # rostopic 的时间戳单位为纳秒，统一换算为秒
    if times and times[0] > 1e6:
        times = [t / 1e9 for t in times]

    return times, xs, ys


def compute_metrics(times, xs, ys):
    """计算性能评价指标。"""
    if len(times) < 2:
        return None

    duration = times[-1] - times[0]

    # 路径长度（逐点累加）
    dist = 0.0
    for i in range(1, len(xs)):
        dist += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])

    # 净位移
    net = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])

    # 瞬时速率序列（用于统计与绘图）
    speeds = []
    speed_times = []
    for i in range(1, len(xs)):
        dt = times[i] - times[i - 1]
        if dt > 1e-9:
            speeds.append(math.hypot(xs[i] - xs[i - 1],
                                     ys[i] - ys[i - 1]) / dt)
            speed_times.append(times[i] - times[0])

    avg_v = dist / duration if duration > 1e-9 else 0.0
    max_v = max(speeds) if speeds else 0.0

    # 航向角序列，用于统计转向总量
    yaws = []
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        if math.hypot(dx, dy) > 1e-6:
            yaws.append(math.atan2(dy, dx))

    total_turn = 0.0
    for i in range(1, len(yaws)):
        d = yaws[i] - yaws[i - 1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        total_turn += abs(d)

    return {
        "duration": duration,
        "distance": dist,
        "net": net,
        "avg_v": avg_v,
        "max_v": max_v,
        "total_turn": total_turn,
        "points": len(times),
        "efficiency": net / dist if dist > 1e-9 else 0.0,
        "x_range": (min(xs), max(xs)),
        "y_range": (min(ys), max(ys)),
        "speeds": speeds,
        "speed_times": speed_times,
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    files = sys.argv[1:]

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        # 中文字体配置：若系统未安装中文字体，图中文字会显示为方框。
        # 这里优先尝试常见的中文字体，缺失时静默回退到默认字体，
        # 并通过关闭 unicode_minus 避免负号显示异常。
        matplotlib.rcParams["axes.unicode_minus"] = False
        for _font in ["WenQuanYi Micro Hei", "Noto Sans CJK JP",
                      "Noto Sans CJK SC", "SimHei", "DejaVu Sans"]:
            try:
                matplotlib.font_manager.findfont(_font, fallback_to_default=False)
                matplotlib.rcParams["font.sans-serif"] = [_font]
                break
            except Exception:  # noqa: BLE001
                continue
    except ImportError:
        print("错误：需要 matplotlib。请执行 sudo apt install python3-matplotlib")
        sys.exit(1)

    results = {}

    print("")
    print("=" * 84)
    print("              OpenHUTB 无人车 —— 轨迹性能评价")
    print("=" * 84)
    print("%-12s %9s %9s %9s %9s %9s %9s" % (
        "算法/模式", "时长(s)", "路程(m)", "位移(m)", "均速(m/s)",
        "峰值(m/s)", "转向(rad)"))
    print("-" * 84)

    for path in files:
        if not os.path.isfile(path):
            print("跳过（文件不存在）：%s" % path)
            continue

        times, xs, ys = load_csv(path)
        m = compute_metrics(times, xs, ys)
        if m is None:
            print("跳过（数据不足）：%s" % path)
            continue

        label = os.path.splitext(os.path.basename(path))[0]
        results[label] = m

        print("%-12s %9.2f %9.2f %9.2f %9.3f %9.3f %9.2f" % (
            label, m["duration"], m["distance"], m["net"],
            m["avg_v"], m["max_v"], m["total_turn"]))

    if not results:
        print("无比可用数据")
        sys.exit(1)

    print("=" * 84)

    # ------------------------------------------------------------------
    # 路径效率说明
    # ------------------------------------------------------------------
    print("")
    print("路径效率 = 净位移 / 实际路程，越接近 1 表示路径越直：")
    for label, m in results.items():
        print("  %-12s %.3f   （X 范围 %.2f~%.2f，Y 范围 %.2f~%.2f）" % (
            label, m["efficiency"],
            m["x_range"][0], m["x_range"][1],
            m["y_range"][0], m["y_range"][1]))
    print("")

    # ------------------------------------------------------------------
    # 图 1：XY 轨迹对比
    # ------------------------------------------------------------------
    fig1, ax1 = plt.subplots(figsize=(10, 10))
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd"]
    for i, path in enumerate(files):
        label = os.path.splitext(os.path.basename(path))[0]
        if label not in results:
            continue
        times, xs, ys = load_csv(path)
        c = colors[i % len(colors)]
        ax1.plot(xs, ys, linewidth=2.0, color=c, label=label)
        ax1.plot(xs[0], ys[0], "o", color=c, markersize=10,
                 markeredgecolor="black", zorder=5)
        ax1.plot(xs[-1], ys[-1], "s", color=c, markersize=10,
                 markeredgecolor="black", zorder=5)
        ax1.annotate("start", (xs[0], ys[0]), fontsize=8,
                     xytext=(6, 6), textcoords="offset points")
        ax1.annotate("end", (xs[-1], ys[-1]), fontsize=8,
                     xytext=(6, -12), textcoords="offset points")

    # 叠加场景中的静态障碍物轮廓（便于理解避障行为）
    obstacles_circle = [(2.5, 2.0), (-2.5, 2.0), (0.0, -2.0)]
    for (ox, oy) in obstacles_circle:
        ax1.add_patch(plt.Circle((ox, oy), 0.4, color="red",
                                 alpha=0.35, zorder=1))
    ax1.add_patch(plt.Rectangle((3.0, -3.5), 1.0, 1.0, color="blue",
                                alpha=0.3, zorder=1))

    # 场地围墙
    ax1.plot([-6, 6, 6, -6, -6], [-6, -6, 6, 6, -6],
             color="gray", linewidth=2, zorder=0)

    ax1.set_xlabel("X (m)", fontsize=11)
    ax1.set_ylabel("Y (m)", fontsize=11)
    ax1.set_title("OpenHUTB UGV — Trajectory Comparison\n"
                  "(gray: arena walls, red circles / blue square: obstacles)",
                  fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.axis("equal")
    ax1.legend(loc="upper left", fontsize=10)
    fig1.tight_layout()
    fig1.savefig("trajectory_plot.png", dpi=120)
    print("轨迹对比图已保存：trajectory_plot.png")

    # ------------------------------------------------------------------
    # 图 2：速度曲线
    # ------------------------------------------------------------------
    fig2, ax2 = plt.subplots(figsize=(11, 6))
    for i, label in enumerate(results.keys()):
        m = results[label]
        c = colors[i % len(colors)]
        ax2.plot(m["speed_times"], m["speeds"], linewidth=1.3,
                 color=c, label=label, alpha=0.85)
    ax2.set_xlabel("time (s)", fontsize=11)
    ax2.set_ylabel("speed (m/s)", fontsize=11)
    ax2.set_title("Speed Profile Comparison", fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best", fontsize=10)
    fig2.tight_layout()
    fig2.savefig("speed_profile.png", dpi=120)
    print("速度曲线图已保存：speed_profile.png")

    # ------------------------------------------------------------------
    # 文本报告
    # ------------------------------------------------------------------
    with open("metrics_report.txt", "w") as fp:
        fp.write("OpenHUTB UGV — Performance Metrics Report\n")
        fp.write("=" * 74 + "\n")
        fp.write("%-12s %9s %9s %9s %9s %9s\n" % (
            "mode", "dur(s)", "dist(m)", "net(m)", "avgv", "maxv"))
        fp.write("-" * 74 + "\n")
        for label, m in results.items():
            fp.write("%-12s %9.2f %9.2f %9.2f %9.3f %9.3f\n" % (
                label, m["duration"], m["distance"], m["net"],
                m["avg_v"], m["max_v"]))
        fp.write("=" * 74 + "\n")
    print("指标报告已保存：metrics_report.txt")
    print("")


if __name__ == "__main__":
    main()
