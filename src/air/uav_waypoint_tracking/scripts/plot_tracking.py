#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取实验日志绘制对比图。用法：python3 plot_tracking.py --mission rectangle"""

import argparse
import csv
import glob
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import yaml
except ImportError:      # 极简环境里没有 pyyaml 时退回文档中的默认增益
    yaml = None


def default_gains():
    """params.yaml 里的默认增益，用来挑出“默认增益那一次”的日志"""
    kp, ki, kd = 1.0, 0.05, 0.2
    if yaml is not None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "config", "params.yaml")
        try:
            with open(path, "r", encoding="utf-8") as f:
                pid = (yaml.safe_load(f) or {}).get("pid", {})
            kp = float(pid.get("kp", kp))
            ki = float(pid.get("ki", ki))
            kd = float(pid.get("kd", kd))
        except (OSError, ValueError, AttributeError):
            pass
    return kp, ki, kd


def load(path):
    rows = []
    with open(path, "r") as f:
        for r in csv.DictReader(f):
            rows.append({k: (v if k == "phase" else float(v)) for k, v in r.items()})
    return rows


def load_metrics(path):
    wp, rms = [], {}
    with open(path, "r") as f:
        for row in csv.reader(f):
            if not row:
                continue
            if len(row) == 4 and row[0].strip().isdigit():
                wp.append([float(x) for x in row])
            elif len(row) == 2:
                rms[row[0]] = float(row[1])
    return wp, rms


def polyline(rows):
    pts = []
    for r in rows:
        p = (r["x_ref"], r["y_ref"], r["z_ref"])
        if not pts or pts[-1] != p:
            pts.append(p)
    return pts


def save(fig, out, name):
    fig.tight_layout()
    fig.savefig(os.path.join(out, name), dpi=150)
    plt.close(fig)
    print("saved:", name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", default="rectangle")
    ap.add_argument("--log-dir",
                    default=os.path.expanduser("~/uav_waypoint_tracking_logs"))
    ap.add_argument("--out", default=".")
    ap.add_argument("--pid-log", default=None,
                    help="指定用作主曲线的 PID 日志；默认自动挑与 params.yaml "
                         "默认增益一致的那一次运行")
    a = ap.parse_args()

    if not os.path.isdir(a.out):
        os.makedirs(a.out)

    # 注意：必须排除 *_metrics.csv —— 否则"指标汇总文件"会被当成轨迹日志读进来
    # （它的第一列是文字，会报 could not convert string to float）
    pid_files = sorted(
        p for p in glob.glob(os.path.join(a.log_dir, "pid_%s_*.csv" % a.mission))
        if not p.endswith("_metrics.csv"))
    bfile = os.path.join(a.log_dir, "builtin_%s.csv" % a.mission)
    if not pid_files:
        print("no pid log found:", os.path.join(a.log_dir, "pid_%s_*.csv" % a.mission))
        return

    # 主曲线要取“默认增益”那一次运行：日志名里带 kp/ki/kd，若直接取最新的一份，
    # 增益实验里最后跑的强增益组会把默认组顶掉，曲线就和文档表格（默认增益）对不上了。
    default_tag = "kp%.2f_ki%.2f_kd%.2f" % default_gains()
    if a.pid_log:
        main_pid = a.pid_log
    else:
        preferred = [p for p in pid_files
                     if os.path.basename(p).endswith(default_tag + ".csv")]
        main_pid = max(preferred or pid_files, key=os.path.getmtime)
        if not preferred:
            print("警告：没有找到默认增益（%s）的日志，暂用 %s"
                  % (default_tag, os.path.basename(main_pid)))
    pid = load(main_pid)
    bui = load(bfile) if os.path.exists(bfile) else None
    print("pid log :", main_pid)
    print("builtin :", bfile if bui else "(none)")

    # 图 1：轨迹对比
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    pts = polyline(pid)
    ax[0].plot([p[0] for p in pts], [p[1] for p in pts], "k--o",
               markersize=4, label="desired")
    ax[0].plot([r["x"] for r in pid], [r["y"] for r in pid], "tab:blue", label="PID")
    if bui:
        ax[0].plot([r["x"] for r in bui], [r["y"] for r in bui],
                   "tab:orange", label="AirSim built-in")
    ax[0].set_xlabel("X (m, east)")
    ax[0].set_ylabel("Y (m, north)")
    ax[0].set_title("Trajectory (XY plane)")
    ax[0].axis("equal")
    ax[0].grid(True)
    ax[0].legend()

    ax[1].plot([r["t"] for r in pid], [r["z"] for r in pid], "tab:blue", label="PID")
    if bui:
        ax[1].plot([r["t"] for r in bui], [r["z"] for r in bui],
                   "tab:orange", label="AirSim built-in")
    ax[1].plot([r["t"] for r in pid], [r["z_ref"] for r in pid], "k--", label="desired")
    ax[1].set_xlabel("time (s)")
    ax[1].set_ylabel("Z (m, up)")
    ax[1].set_title("Altitude")
    ax[1].grid(True)
    ax[1].legend()
    save(fig, a.out, "trajectory_%s.png" % a.mission)

    # 图 2：位置误差曲线
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot([r["t"] for r in pid], [r["dist"] for r in pid], "tab:blue", label="PID")
    if bui:
        ax.plot([r["t"] for r in bui], [r["dist"] for r in bui],
                "tab:orange", label="AirSim built-in")
    ax.axhline(0.25, color="gray", linestyle=":", label="tolerance 0.25 m")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance to target (m)")
    ax.set_title("Position error")
    ax.grid(True)
    ax.legend()
    save(fig, a.out, "error_%s.png" % a.mission)

    # 图 3：指标柱状对比
    pmet = os.path.splitext(main_pid)[0] + "_metrics.csv"
    bmet = os.path.join(a.log_dir, "builtin_%s_metrics.csv" % a.mission)
    if os.path.exists(pmet):
        pw, pr = load_metrics(pmet)
        bw, br = load_metrics(bmet) if os.path.exists(bmet) else ([], {})
        if pw:
            idx = [int(r[0]) for r in pw]
            w = 0.35
            fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
            ax[0].bar([i - w / 2 for i in idx], [r[1] for r in pw], w,
                      label="PID", color="tab:blue")
            if bw:
                ax[0].bar([i + w / 2 for i in idx], [r[1] for r in bw], w,
                          label="AirSim built-in", color="tab:orange")
            ax[0].set_xlabel("waypoint")
            ax[0].set_ylabel("arrival time (s)")
            ax[0].set_title("Arrival time")
            ax[0].grid(True, axis="y")
            ax[0].legend()

            ax[1].bar([i - w / 2 for i in idx], [r[2] for r in pw], w,
                      label="PID", color="tab:blue")
            if bw:
                ax[1].bar([i + w / 2 for i in idx], [r[2] for r in bw], w,
                          label="AirSim built-in", color="tab:orange")
            ax[1].set_xlabel("waypoint")
            ax[1].set_ylabel("steady-state error (m)")
            ax[1].set_title("Steady-state error")
            ax[1].grid(True, axis="y")
            ax[1].legend()
            save(fig, a.out, "metrics_%s.png" % a.mission)

            print("")
            print("======== metric comparison ========")
            print("wp   PID_arr(s)  builtin_arr(s)  PID_sse(m)  builtin_sse(m)")
            for i, r in enumerate(pw):
                b = bw[i] if i < len(bw) else [0, float("nan"), float("nan"), float("nan")]
                print("%2d   %9.2f  %13.2f  %10.3f  %14.3f" %
                      (int(r[0]), r[1], b[1], r[2], b[2]))
            print("RMS  PID: all %.3f | horizontal %.3f | vertical %.3f" %
                  (pr.get("trajectory_rms_all_m", float("nan")),
                   pr.get("trajectory_rms_horizontal_m", float("nan")),
                   pr.get("trajectory_rms_vertical_m", float("nan"))))
            if br:
                print("RMS  built-in: all %.3f | horizontal %.3f | vertical %.3f" %
                      (br.get("trajectory_rms_all_m", float("nan")),
                       br.get("trajectory_rms_horizontal_m", float("nan")),
                       br.get("trajectory_rms_vertical_m", float("nan"))))
            print("===================================")
            print("")

    # 图 4：不同增益对比
    if len(pid_files) > 1:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for path in pid_files:
            rows = load(path)
            m = re.findall(r"kp([0-9.]+)_ki([0-9.]+)_kd([0-9.]+)", os.path.basename(path))
            label = ("kp=%s ki=%s kd=%s" % m[0]) if m else os.path.basename(path)
            ax.plot([r["t"] for r in rows], [r["dist"] for r in rows], label=label)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("distance to target (m)")
        ax.set_title("PID gain tuning comparison")
        ax.grid(True)
        ax.legend(fontsize=8)
        save(fig, a.out, "gains_%s.png" % a.mission)

    print("output dir:", os.path.abspath(a.out))


if __name__ == "__main__":
    main()
