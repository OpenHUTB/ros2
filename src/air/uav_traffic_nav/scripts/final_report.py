#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最终报告：所有策略在同一协议下对比（难度 1.0，100 回合，seed 11）."""
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
SEED = 11
CASES = [
    ("rule expert (privileged)", lambda: (lambda e, o: expert_action(e)), None),
    ("BC + DAgger", None, "models/bc_policy_s42.npz"),
    ("BC + DAgger + PPO", None, "models/pol_bc_ppo.npz"),
    ("PPO from scratch", None, "models/ppo_scratch.npz"),
    ("greedy", lambda: (lambda e, o: BASELINES["greedy"](e)), None),
    ("velocity obstacles", lambda: (lambda e, o: BASELINES["vo"](e)), None),
]


def main():
    os.makedirs("results", exist_ok=True)
    rows = []
    for name, factory, path in CASES:
        fn = factory() if factory else _policy_fn(path)
        for forced in (False, True):
            row = evaluate(name, fn, EPISODES, difficulty=1.0, seed=SEED,
                           include_traffic=True, force_crossing=forced)
            row["distribution"] = "forced" if forced else "mixed"
            rows.append(row)
            print("%-24s %-6s success %3.0f%%  crash %3.0f%%" % (
                name, row["distribution"], 100 * row["success_rate"],
                100 * row["crash_rate"]))

    with open("results/final_report.json", "w", encoding="utf-8") as fh:
        json.dump(rows, fh, ensure_ascii=False, indent=2)

    names = [c[0] for c in CASES]
    mixed = [100 * r["success_rate"] for r in rows if r["distribution"] == "mixed"]
    forced = [100 * r["success_rate"] for r in rows if r["distribution"] == "forced"]
    x = np.arange(len(names))
    fig, ax = plt.subplots(figsize=(9.5, 4.4))
    ax.bar(x - 0.2, mixed, width=0.4, label="mixed start/goal")
    ax.bar(x + 0.2, forced, width=0.4, label="forced crossing")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, fontsize=8)
    ax.set_ylabel("success rate (%)")
    ax.set_title("difficulty 1.0, %d episodes per condition" % EPISODES)
    ax.grid(axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig("results/final_report.png", dpi=150)

    lines = ["| 策略 | 混合起终点 | 强制跨路 |", "| --- | --- | --- |"]
    for i, name in enumerate(names):
        lines.append("| %s | %.0f%% | %.0f%% |" % (name, mixed[i], forced[i]))
    table = "\n".join(lines) + "\n"
    with open("results/final_report.md", "w", encoding="utf-8") as fh:
        fh.write(table)
    print(table)


if __name__ == "__main__":
    main()
