#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多评估种子稳健性测量：同一权重在不同 eval seed 下表现差多少.

为什么需要：评审指出「种子 1 那行的 80.0%」其实来自训练期的 eval seed=1001，
而主表的 76.7% 来自 eval seed=1000 —— 两个数不是同一把尺子，
读者复现时会拿不到 80%。这里把多个 eval seed 都跑一遍，给出均值±标准差。
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import policy as P
from route_env import RouteEnv


def eval_seed(pol, seed: int, episodes: int = 30, obstacles: int = 14) -> dict:
    env = RouteEnv(seed=seed, num_obstacles=obstacles)
    ok = crash = timeout = 0
    crosses = []
    for k in range(episodes):
        # 与 eval.py 的回合种子约定保持一致：seed + 回合序号
        env.reset(seed=seed + k)
        done = False
        info = {}
        cs = []
        while not done:
            a = pol.forward(env._obs())
            _o, _r, term, trunc, info = env.step(a)
            _, _s, _, c = env.route.project(env.pos[:2])
            cs.append(abs(c))
            done = bool(term or trunc)
        crosses.append(float(np.mean(cs)))
        if info.get("success"):
            ok += 1
        elif info.get("crash_reason"):
            crash += 1
        else:
            timeout += 1
    n = float(max(episodes, 1))
    return {"seed": seed, "lap": ok / n * 100, "crash": crash / n * 100,
            "timeout": timeout / n * 100, "cross": float(np.mean(crosses))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="多评估种子稳健性测量")
    ap.add_argument("--models", nargs="+",
                    default=["models/route_ppo_s0.npz", "models/route_ppo_s1.npz"])
    ap.add_argument("--seeds", default="0,1,2,3,4,5,1000")
    ap.add_argument("--episodes", type=int, default=30)
    args = ap.parse_args(argv)

    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    print("=" * 78)
    print("  多评估种子稳健性测量（每个 eval seed 跑 %d 回合，14 个障碍）" % args.episodes)
    print("=" * 78)
    for mp in args.models:
        if not os.path.isfile(mp):
            print("  [跳过] 找不到 %s" % mp)
            continue
        pol = P.MlpPolicy.from_npz(mp)
        rows = [eval_seed(pol, sd, args.episodes) for sd in seeds]
        laps = np.array([r["lap"] for r in rows])
        crs = np.array([r["crash"] for r in rows])
        print()
        print("  ---- %s ----" % os.path.basename(mp))
        print("  %-8s %-10s %-10s %-10s" % ("eval种子", "完成一圈%", "碰撞%", "横向偏差m"))
        for r in rows:
            print("  %-8d %-10.3f %-10.3f %-10.3f"
                  % (r["seed"], r["lap"], r["crash"], r["cross"]))
        print("  " + "-" * 44)
        print("  均值±标准差   %.1f ± %.1f     %.1f ± %.1f"
              % (laps.mean(), laps.std(ddof=1) if len(laps) > 1 else 0.0,
                 crs.mean(), crs.std(ddof=1) if len(crs) > 1 else 0.0))
        print("  最小值 / 最大值   %.1f%% / %.1f%%" % (laps.min(), laps.max()))
    print()
    print("=" * 78)
    print("  结论：同一个权重换评估种子，完成率会在上面这个区间里波动 ——")
    print("  所以对外只能说区间与均值，不能说单个最好看的数。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
