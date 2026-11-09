#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取编队实验日志，绘制队形误差曲线、航迹俯视图与指标柱状图。

用法：
  rosrun uav_formation_flight plot_formation.py --mission line_straight \
      --out ~/uav_formation_flight_logs/figs
"""

import argparse
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_samples(path):
    rows = []
    with open(path, "r") as fh:
        for row in csv.DictReader(fh):
            rows.append(row)
    return rows


def read_metrics(path):
    out = []
    with open(path, "r") as fh:
        for row in csv.DictReader(fh):
            out.append(row)
    return out


def find_logs(log_dir, mission):
    pattern = os.path.join(log_dir, "formation_%s_kp*_metrics.csv" % mission)
    metrics = sorted(glob.glob(pattern))
    if not metrics:
        raise SystemExit("没有找到指标文件：%s" % pattern)
    tag = os.path.basename(metrics[-1])[:-len("_metrics.csv")]
    samples = os.path.join(log_dir, tag + ".csv")
    if not os.path.isfile(samples):
        raise SystemExit("没有找到采样文件：%s" % samples)
    return samples, metrics[-1]


def plot_error_curve(rows, out):
    vehicles = sorted({r["vehicle"] for r in rows})
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for name in vehicles:
        pts = [(float(r["t"]), float(r["dist"]))
               for r in rows if r["vehicle"] == name]
        if not pts:
            continue
        xs, ys = zip(*pts)
        ax.plot(xs, ys, linewidth=1.6, label=name)
    ax.axhline(0.30, color="crimson", linestyle="--", linewidth=1.0,
               label="0.30 m 容差")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("队形误差 (m)")
    ax.set_title("队形误差随时间变化")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_tracks(rows, out, leader):
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    lead_ref = [(float(r["x_ref"]), float(r["y_ref"]))
                for r in rows if r["vehicle"] == leader]
    if lead_ref:
        ax.plot([p[0] for p in lead_ref], [p[1] for p in lead_ref],
                "k--", linewidth=1.2, label="长机期望航迹")
    for name in sorted({r["vehicle"] for r in rows}):
        pts = [(float(r["x"]), float(r["y"]))
               for r in rows if r["vehicle"] == name]
        if not pts:
            continue
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                linewidth=1.5, label=name)
    ax.set_xlabel("东向 x (m)")
    ax.set_ylabel("北向 y (m)")
    ax.set_title("编队航迹俯视图")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_metrics(metrics, out):
    rms = [m for m in metrics if m["metric"] == "formation_rms" and m["vehicle"]]
    mx = [m for m in metrics if m["metric"] == "formation_max" and m["vehicle"]]
    if not rms:
        return
    names = [m["vehicle"] for m in rms]
    xs = range(len(names))
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    width = 0.38
    ax.bar([x - width / 2 for x in xs],
           [float(m["value"]) for m in rms], width, label="RMS 队形误差")
    ax.bar([x + width / 2 for x in xs],
           [float(m["value"]) for m in mx], width, label="最大队形误差")
    ax.axhline(0.30, color="crimson", linestyle="--", linewidth=1.0)
    ax.set_xticks(list(xs))
    ax.set_xticklabels(names, fontsize=8)
    ax.set_ylabel("误差 (m)")
    ax.set_title("各僚机的队形保持误差")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="编队实验出图")
    ap.add_argument("--mission", required=True)
    ap.add_argument("--leader", default="SimpleFlight",
                    help="长机载具名，用于区分期望航迹")
    ap.add_argument("--log-dir", default=os.path.expanduser(
        "~/uav_formation_flight_logs"))
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    samples, metric_path = find_logs(args.log_dir, args.mission)
    rows = read_samples(samples)
    metrics = read_metrics(metric_path)
    if not os.path.isdir(args.out):
        os.makedirs(args.out)

    plot_error_curve(rows, os.path.join(args.out, "formation_error.png"))
    plot_tracks(rows, os.path.join(args.out, "formation_tracks.png"), args.leader)
    plot_metrics(metrics, os.path.join(args.out, "formation_metrics.png"))

    print("saved: formation_error.png / formation_tracks.png / formation_metrics.png")
    for metric, vehicle, value, unit in [
            (m["metric"], m["vehicle"], m["value"], m["unit"]) for m in metrics]:
        label = metric + ("[%s]" % vehicle if vehicle else "")
        print("  %-28s %8s %s" % (label, value, unit))


if __name__ == "__main__":
    main()
