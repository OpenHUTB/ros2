#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评估训练好的策略：成功率 / 失败率 / 平均步数 / 平均奖励."""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from env import NavEnv
from policy import MlpPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", type=str, default="models/policy_weights.npz")
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument("--obstacles", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    policy = MlpPolicy.from_npz(args.weights)
    env = NavEnv(num_obstacles=args.obstacles, seed=args.seed)

    succ = crash = timeout = 0
    rewards, steps_list = [], []
    for _ in range(args.episodes):
        obs, _ = env.reset()
        done = False
        ep_r, n = 0.0, 0
        while not done:
            act = policy.forward(obs)
            obs, r, terminated, truncated, info = env.step(act)
            ep_r += r
            n += 1
            done = terminated or truncated
        rewards.append(ep_r)
        steps_list.append(n)
        if info["success"]:
            succ += 1
        elif info["crash_reason"]:
            crash += 1
        else:
            timeout += 1

    print("成功率: %d/%d (%.1f%%)" % (succ, args.episodes, 100.0 * succ / args.episodes))
    print("失败(碰撞/越界): %d, 超时: %d" % (crash, timeout))
    print("平均步数: %.1f, 平均奖励: %.1f" % (np.mean(steps_list), np.mean(rewards)))
    env.close()


if __name__ == "__main__":
    main()
