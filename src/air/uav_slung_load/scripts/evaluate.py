#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""吊挂负载控制对比评测：三种控制律 × 三种负载质量。

场景：先在起点悬停 1 s，然后给 2 m 阶跃；记录 12 s 的摆角与位置指标。
可选 ``--with-gust``：阶跃后在第 8 s 给负载一个 0.1 s 的横向阵风（1.5 N）。

输出：终端表格 + ``results/eval_<时间戳>.csv``。
"""

import argparse
import csv
import datetime
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controllers import CONTROLLERS  # noqa: E402
from sim_core import SlungSim, run_episode, summarize  # noqa: E402

FIELDS = ["scenario", "payload_kg", "controller", "swing_rms_deg", "swing_max_deg",
          "residual_deg", "pos_rms_m", "pos_final_m", "settle_s", "thrust_max_n"]


def evaluate(seconds=12.0, masses=(0.1, 0.3, 0.5), target=(2.0, 0.0, 1.5),
             step_time=1.0, with_gust=False, controllers=("pd", "zv", "swing")):
    rows = []
    for mass in masses:
        sim = SlungSim(payload_mass=mass)
        for name in controllers:
            ctrl = CONTROLLERS[name]()
            log = run_episode(ctrl, target, seconds, payload_mass=mass, sim=sim,
                              step_time=step_time)
            stats = summarize(log, settle_window=2.0, t_start=step_time)
            rows.append(dict(scenario="step", payload_kg=mass, controller=name, **stats))

            if with_gust:
                # 阵风：1.5 N × 0.1 s 的横向冲量（约 0.15 N·s），量级相当于一次轻碰
                gust = (sim.payload_id, [1.5, 0.0, 0.0], 8.0, 0.1)
                log = run_episode(ctrl, target, seconds, payload_mass=mass, sim=sim,
                                  step_time=step_time, gust=gust)
                stats = summarize(log, settle_window=2.0, t_start=step_time)
                rows.append(dict(scenario="gust", payload_kg=mass, controller=name, **stats))
    return rows


def print_table(rows):
    head = "%-5s %-6s %-6s %8s %8s %9s %8s %8s %7s" % (
        "场景", "负载", "控制", "摆角RMS", "摆角峰值", "残余摆角", "位置RMS", "末端误差", "调节时间")
    print(head)
    print("-" * len(head) + "-" * 6)
    for r in rows:
        settle = "nan" if np.isnan(r["settle_s"]) else "%.2f" % r["settle_s"]
        print("%-7s %-7.1f %-8s %7.2f° %7.2f° %8.2f° %7.3f %7.3f %8s" % (
            r["scenario"], r["payload_kg"], r["controller"], r["swing_rms_deg"],
            r["swing_max_deg"], r["residual_deg"], r["pos_rms_m"], r["pos_final_m"],
            settle))


def main(argv=None):
    parser = argparse.ArgumentParser(description="吊挂负载防摆控制对比评测")
    parser.add_argument("--seconds", type=float, default=12.0, help="每段仿真时长（s）")
    parser.add_argument("--masses", type=str, default="0.1,0.3,0.5", help="负载质量列表（kg）")
    parser.add_argument("--target", type=str, default="2.0,0.0,1.5", help="阶跃目标 x,y,z")
    parser.add_argument("--step-time", type=float, default=1.0, help="阶跃时刻（s）")
    parser.add_argument("--with-gust", action="store_true", help="额外跑阵风扰动场景")
    parser.add_argument("--out", type=str, default="", help="CSV 输出路径（默认 results/）")
    args = parser.parse_args(argv)

    masses = tuple(float(v) for v in args.masses.split(","))
    target = tuple(float(v) for v in args.target.split(","))
    rows = evaluate(args.seconds, masses, target, args.step_time, args.with_gust)

    print_table(rows)

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                   "results", "eval_%s.csv" % stamp)
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r[k] for k in FIELDS})
    print("\nCSV 已保存：%s" % out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
