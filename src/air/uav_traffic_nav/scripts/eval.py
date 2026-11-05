#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""策略评估：成功率 / 碰撞率 / 最小间距 / TTC 违例 / 路径长度 / 耗时.

用法：
    python3 eval.py --model models/bc_policy_s42.npz --episodes 100
    python3 eval.py --compare --episodes 100 --force-crossing --out results
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from baselines import BASELINES
from policy import MlpPolicy
from traffic_env import TrafficNavEnv


def _policy_fn(model_path):
    policy = MlpPolicy.from_npz(model_path)
    return lambda env, obs: policy.forward(obs)


def evaluate(policy_name, policy_fn, episodes=50, difficulty=1.0, seed=7,
             include_traffic=True, max_steps=500, force_crossing=False):
    """跑固定难度下的若干回合，返回指标字典."""
    env = TrafficNavEnv(include_traffic=include_traffic, seed=seed,
                        max_steps=max_steps, force_crossing=force_crossing)
    env.set_difficulty(difficulty)
    success = 0
    crashes = {}
    timeouts = 0
    clearances = []
    ttcs = []
    times = []
    lengths = []
    for episode in range(episodes):
        obs, _ = env.reset(seed=seed * 1000 + episode)
        done = False
        path = 0.0
        last = env.pos.copy()
        steps = 0
        min_clear = float("inf")
        min_ttc = float("inf")
        while not done:
            action = policy_fn(env, obs)
            obs, _reward, terminated, truncated, info = env.step(action)
            steps += 1
            path += float(np.linalg.norm(env.pos - last))
            last = env.pos.copy()
            min_clear = min(min_clear, info.get("min_clearance", float("inf")))
            if not math.isnan(info.get("min_ttc", float("nan"))):
                min_ttc = min(min_ttc, info["min_ttc"])
            done = terminated or truncated
        if info.get("success"):
            success += 1
            times.append(steps * env.dt)
        elif info.get("crash_reason"):
            crashes[info["crash_reason"]] = crashes.get(info["crash_reason"], 0) + 1
        else:
            timeouts += 1
        clearances.append(min_clear)
        lengths.append(path)
        if min_ttc < float("inf"):
            ttcs.append(min_ttc)
    env.close()
    return {
        "policy": policy_name,
        "episodes": episodes,
        "difficulty": difficulty,
        "force_crossing": force_crossing,
        "success_rate": success / episodes,
        "crash_rate": sum(crashes.values()) / episodes,
        "timeout_rate": timeouts / episodes,
        "crash_reasons": crashes,
        "mean_min_clearance_m": float(np.mean(clearances)) if clearances else float("nan"),
        "min_clearance_m": float(np.min(clearances)) if clearances else float("nan"),
        "mean_min_ttc_s": float(np.mean(ttcs)) if ttcs else float("nan"),
        "ttc_violations": sum(1 for t in ttcs if t < 0.5),
        "mean_time_s": float(np.mean(times)) if times else float("nan"),
        "mean_path_m": float(np.mean(lengths)) if lengths else float("nan"),
    }


def format_table(rows):
    head = ("policy", "succ", "crash", "timeout", "clear(m)", "minTTC(s)", "path(m)")
    lines = ["%-24s %6s %6s %8s %9s %10s %8s" % head]
    for r in rows:
        lines.append("%-24s %6.1f %6.1f %8.1f %9.2f %10.2f %8.1f" % (
            r["policy"], 100 * r["success_rate"], 100 * r["crash_rate"],
            100 * r["timeout_rate"], r["mean_min_clearance_m"],
            r["mean_min_ttc_s"], r["mean_path_m"]))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="评估动态交通避障策略")
    ap.add_argument("--model", default="models/bc_policy_s42.npz")
    ap.add_argument("--baseline", choices=sorted(BASELINES), default=None)
    ap.add_argument("--compare", action="store_true", help="对比已训练模型与基线")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--difficulty", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--force-crossing", action="store_true",
                    help="评测分布：起点/终点固定在车道两侧（强制横穿车流）")
    ap.add_argument("--out", default="results")
    args = ap.parse_args()

    rows = []
    if args.compare:
        candidates = []
        for name in ("bc_policy_s42", "ppo_scratch"):
            path = os.path.join("models", name + ".npz")
            if os.path.exists(path):
                candidates.append((name, _policy_fn(path)))
        for name, fn in BASELINES.items():
            candidates.append((name, lambda env, obs, fn=fn: fn(env)))
        for name, fn in candidates:
            include = not name.endswith("lidar_only")
            rows.append(evaluate(name, fn, args.episodes, args.difficulty,
                                 args.seed, include_traffic=include,
                                 force_crossing=args.force_crossing))
    elif args.baseline:
        rows.append(evaluate(args.baseline, lambda env, obs: BASELINES[args.baseline](env),
                             args.episodes, args.difficulty, args.seed,
                             force_crossing=args.force_crossing))
    else:
        include = not args.model.endswith("lidar_only.npz")
        rows.append(evaluate(os.path.basename(args.model), _policy_fn(args.model),
                             args.episodes, args.difficulty, args.seed,
                             include_traffic=include, force_crossing=args.force_crossing))

    print(format_table(rows))
    os.makedirs(args.out, exist_ok=True)
    out = os.path.join(args.out, "eval_%s.json" % ("compare" if args.compare
                                                   else rows[0]["policy"]))
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(rows, fh, ensure_ascii=False, indent=2)
    print("结果已写入 %s" % out)


if __name__ == "__main__":
    main()
