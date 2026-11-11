#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四足步态参数搜索：找出"不翻倒且能前进"的参数，并输出对照表。

搜索维度：步态频率、大腿幅度、小腿幅度、相位差，以及三个反馈增益。
评价准则（阈值与 locomotion_core 一致）：
    翻倒 = 最大倾角 > 60° 或 基座最低高度 < 0.15 m
    未翻倒时 得分 = 前进位移 − 2 × |横向漂移|

示例::

    python3 tune_gait.py --trials 150 --seconds 4 --seed 0
    python3 tune_gait.py --trials 300 --seconds 4 --csv /tmp/gait_scan.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import sys

from locomotion_core import QuadrupedModel, TrotGait, default_model_path, simulate


def sample_params(rng):
    return dict(
        freq=rng.uniform(1.5, 4.0),
        a_thigh=rng.uniform(0.05, 0.30),
        a_calf=rng.uniform(0.05, 0.40),
        phase_offset=rng.choice([0.0, math.pi / 2, math.pi]),
        kp_roll=rng.choice([0.0, 0.01, 0.02, 0.05]),
        kp_pitch=rng.choice([0.0, 0.01, 0.02, 0.05]),
        kp_height=rng.choice([0.0, 0.25, 0.5, 1.0]),
    )


def main(argv=None):
    p = argparse.ArgumentParser(description="四足步态参数搜索")
    p.add_argument("--model", default=default_model_path())
    p.add_argument("--trials", type=int, default=150)
    p.add_argument("--seconds", type=float, default=4.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--top", type=int, default=8)
    p.add_argument("--csv", default=None, help="保存全部搜索结果")
    ns = p.parse_args(sys.argv[1:] if argv is None else argv)

    robot = QuadrupedModel(ns.model)
    rng = random.Random(ns.seed)
    rows = []

    for _ in range(ns.trials):
        params = sample_params(rng)
        controller = TrotGait(robot, **params)
        m = simulate(robot, controller, seconds=ns.seconds)
        score = -999.0 if m["flipped"] else m["dx"] - 2.0 * abs(m["dy"])
        rows.append(dict(params=params, metrics=m, score=score))

    rows.sort(key=lambda r: -r["score"])
    ok = [r for r in rows if not r["metrics"]["flipped"]]
    moving = [r for r in ok if r["metrics"]["dx"] > 0.2]

    print("## 搜索概览\n")
    print("- 试验总数: %d" % ns.trials)
    print("- 未翻倒: %d (%.0f%%)" % (len(ok), 100.0 * len(ok) / ns.trials))
    print("- 未翻倒且前进 > 0.2 m: %d (%.0f%%)"
          % (len(moving), 100.0 * len(moving) / ns.trials))
    print("- 速度 (%.1f s): 前进位移按秒计" % ns.seconds)

    print("\n## Top %d（按 前进 − 2×|横漂| 排序）\n" % ns.top)
    print("| # | freq | thigh | calf | phase | kp_roll | kp_pitch | kp_h | "
          "Δx | Δy | max_tilt | min_z | 翻倒 | 得分 |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for i, r in enumerate(rows[:ns.top]):
        q, m = r["params"], r["metrics"]
        print("| %d | %.2f | %.3f | %.3f | %.2f | %s | %s | %s | %+.3f | %+.3f | "
              "%.1f° | %.3f | %s | %+.3f |"
              % (i + 1, q["freq"], q["a_thigh"], q["a_calf"], q["phase_offset"],
                 q["kp_roll"], q["kp_pitch"], q["kp_height"],
                 m["dx"], m["dy"], m["max_tilt_deg"], m["min_base_height"],
                 "是" if m["flipped"] else "否", r["score"]))

    if ns.csv:
        with open(ns.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["freq", "a_thigh", "a_calf", "phase_offset", "kp_roll",
                        "kp_pitch", "kp_height", "dx", "dy", "max_tilt_deg",
                        "min_base_height", "flipped", "score"])
            for r in rows:
                q, m = r["params"], r["metrics"]
                w.writerow([q["freq"], q["a_thigh"], q["a_calf"],
                            q["phase_offset"], q["kp_roll"], q["kp_pitch"],
                            q["kp_height"], m["dx"], m["dy"],
                            m["max_tilt_deg"], m["min_base_height"],
                            int(m["flipped"]), r["score"]])
        print("\n全部结果已写入: %s" % ns.csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
