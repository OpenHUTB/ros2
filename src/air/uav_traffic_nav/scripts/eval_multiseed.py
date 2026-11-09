#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多种子报告：DAgger 3 个训练种子 × 两种分布，对比基线与专家上界.

输出 results/multiseed.json / multiseed.md / multiseed.png。
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from baselines import BASELINES, expert_action
from eval import evaluate, _policy_fn

EPISODES = 100
RESULTS_DIR = "results"
EVAL_SEED = 11
SEEDS = (42, 43, 44)


def run(fn, forced):
    return evaluate("x", fn, EPISODES, difficulty=1.0, seed=EVAL_SEED,
                    include_traffic=True, force_crossing=forced)


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    rows = []
    for seed in SEEDS:
        path = "models/bc_policy_s%d.npz" % seed
        if not os.path.exists(path):
            print("缺少 %s" % path)
            continue
        fn = _policy_fn(path)
        for forced in (False, True):
            r = run(fn, forced)
            r["policy"] = "BC + DAgger (seed %d)" % seed
            r["distribution"] = "forced" if forced else "mixed"
            r["seed"] = seed
            rows.append(r)
            print("%-26s %-6s success %3.0f%%  crash %3.0f%%" % (
                r["policy"], r["distribution"], 100 * r["success_rate"],
                100 * r["crash_rate"]))

    extra = []
    for name, fn in (("greedy", lambda e, o: BASELINES["greedy"](e)),
                     ("velocity obstacles", lambda e, o: BASELINES["vo"](e)),
                     ("rule expert (privileged)", lambda e, o: expert_action(e))):
        for forced in (False, True):
            r = run(fn, forced)
            r["policy"] = name
            r["distribution"] = "forced" if forced else "mixed"
            extra.append(r)
            print("%-26s %-6s success %3.0f%%" % (name, r["distribution"],
                                                  100 * r["success_rate"]))

    # 汇总
    summary = []
    for dist in ("mixed", "forced"):
        values = [r["success_rate"] for r in rows if r["distribution"] == dist]
        crashes = [r["crash_rate"] for r in rows if r["distribution"] == dist]
        if values:
            summary.append({"group": "BC + DAgger", "distribution": dist,
                            "n": len(values),
                            "success_mean": float(np.mean(values)),
                            "success_std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                            "crash_mean": float(np.mean(crashes))})
        for name in ("greedy", "velocity obstacles", "rule expert (privileged)"):
            hit = [r for r in extra if r["policy"] == name and r["distribution"] == dist]
            if hit:
                summary.append({"group": name, "distribution": dist, "n": 1,
                                "success_mean": hit[0]["success_rate"],
                                "success_std": 0.0, "crash_mean": hit[0]["crash_rate"]})

    with open("results/multiseed.json", "w", encoding="utf-8") as fh:
        json.dump({"runs": rows + extra, "summary": summary}, fh,
                  ensure_ascii=False, indent=2)

    groups = ["BC + DAgger", "greedy", "velocity obstacles", "rule expert (privileged)"]
    mixed = [next(s for s in summary if s["group"] == g and s["distribution"] == "mixed") for g in groups]
    forced = [next(s for s in summary if s["group"] == g and s["distribution"] == "forced") for g in groups]

    lines = ["| 策略 | 混合起终点 | 强制跨路 |", "| --- | --- | --- |"]
    for g, m, f in zip(groups, mixed, forced):
        if g == "BC + DAgger":
            lines.append("| %s（%d 个种子） | %.1f%% ± %.1f%% | %.1f%% ± %.1f%% |" % (
                g, m["n"], 100 * m["success_mean"], 100 * m["success_std"],
                100 * f["success_mean"], 100 * f["success_std"]))
        else:
            lines.append("| %s | %.0f%% | %.0f%% |" % (g, 100 * m["success_mean"],
                                                       100 * f["success_mean"]))
    table = "\n".join(lines) + "\n"
    with open("results/multiseed.md", "w", encoding="utf-8") as fh:
        fh.write(table)
    print(table)

    x = np.arange(len(groups))
    fig, ax = plt.subplots(figsize=(9, 4.3))
    ax.bar(x - 0.2, [100 * m["success_mean"] for m in mixed], width=0.4,
           yerr=[100 * m["success_std"] for m in mixed], capsize=4, label="mixed start/goal")
    ax.bar(x + 0.2, [100 * f["success_mean"] for f in forced], width=0.4,
           yerr=[100 * f["success_std"] for f in forced], capsize=4, label="forced crossing")
    ax.set_xticks(x)
    ax.set_xticklabels(groups, rotation=12, fontsize=8)
    ax.set_ylabel("success rate (%)")
    ax.set_title("difficulty 1.0, %d episodes per condition (mean ± std over %d training seeds)"
                 % (EPISODES, len(SEEDS)))
    ax.grid(axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig("results/multiseed.png", dpi=150)


if __name__ == "__main__":
    main()
