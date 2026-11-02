#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对照评估：PPO 策略 vs「纯循迹」手工控制器（不看障碍），多随机种子.

手工控制器 = 只朝前方航点飞、完全无视障碍 —— 它是「不学避障」的下界，
用来回答一个关键问题：**固定航线巡航里，神经网络到底有没有用？**
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import policy as P
import train as T
from route_env import MAX_HORIZ_SPEED, RouteEnv


def pure_pursuit(env, lookahead: float = 4.0):
    """只看航线、不看障碍的手工控制器（返回动作函数）."""
    state = {"s": 0.0}

    def act(obs):
        _, s, _, _ = env.route.project(env.pos[:2])
        wp = env.route.waypoint_ahead(env.pos[:2], s, lookahead)
        d = wp - env.pos[:2]
        n = np.linalg.norm(d) or 1.0
        a = np.zeros(4, dtype=np.float32)
        a[0] = d[0] / n
        a[1] = d[1] / n
        a[2] = float(np.clip(-(env.pos[2] - env.route.points[0, 2]) / 1.5, -1.0, 1.0))
        return a
    return act


def eval_with(act_factory, seeds, episodes_per_seed=10, **env_kw):
    """按 seed 逐个评估并汇总。"""
    agg = {"lap": 0, "crash": 0, "timeout": 0, "n": 0,
           "cross": [], "comp": [], "steps": []}
    for sd in seeds:
        env = RouteEnv(seed=sd, **env_kw)
        act = act_factory(env)
        for ep in range(episodes_per_seed):
            obs, _ = env.reset(seed=sd * 1000 + ep)
            done = False
            info = {}
            crosses = []
            steps = 0
            while not done:
                a = act(obs)
                obs, _r, term, trunc, info = env.step(a)
                _, _s, _, cross = env.route.project(env.pos[:2])
                crosses.append(abs(cross))
                steps += 1
                done = bool(term or trunc)
            agg["n"] += 1
            agg["steps"].append(steps)
            agg["cross"].append(float(np.mean(crosses)))
            agg["comp"].append(env.route.completion(env.visited_s))
            if info.get("success"):
                agg["lap"] += 1
            elif info.get("crash_reason"):
                agg["crash"] += 1
            else:
                agg["timeout"] += 1
    n = float(agg["n"])
    return {"n": agg["n"], "lap_rate": agg["lap"] / n, "crash_rate": agg["crash"] / n,
            "timeout_rate": agg["timeout"] / n,
            "mean_cross": float(np.mean(agg["cross"])),
            "mean_completion": float(np.mean(agg["comp"])),
            "mean_steps": float(np.mean(agg["steps"]))}


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/route_ppo_s0.npz")
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--obstacles", type=int, default=14)
    args = ap.parse_args(argv)

    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    kw = dict(num_obstacles=args.obstacles)
    pol = P.MlpPolicy.from_npz(args.model)

    print("=" * 70)
    print("  固定航线巡航：PPO 策略 vs 纯循迹手工控制器")
    print("  航线 rect（62.8 m 闭合），每 seed %d 回合，随机障碍 %d 个"
          % (args.episodes, args.obstacles))
    print("=" * 70)

    r_ppo = eval_with(lambda env: (lambda o: pol.forward(o)), seeds, args.episodes, **kw)
    r_pp = eval_with(lambda env: pure_pursuit(env), seeds, args.episodes, **kw)

    rows = [("完成一圈率", "lap_rate", 100.0), ("碰撞率", "crash_rate", 100.0),
            ("超时率", "timeout_rate", 100.0), ("平均横向偏差(m)", "mean_cross", 1.0),
            ("平均完成度(%)", "mean_completion", 100.0), ("平均步数", "mean_steps", 1.0)]
    print()
    print("  %-18s %-16s %-16s" % ("指标", "PPO 策略", "纯循迹(不看障碍)"))
    for label, key, scale in rows:
        print("  %-18s %-16.3f %-16.3f" % (label, r_ppo[key] * scale, r_pp[key] * scale))
    print("  %-18s %-16d %-16d" % ("回合数", r_ppo["n"], r_pp["n"]))
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
