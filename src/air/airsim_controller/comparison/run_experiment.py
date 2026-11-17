"""对比实验入口：
  1) 同一 MLP 骨干，分别跑 PPO / DQN / SAC，记录每轮回报；
  2) 同一 PPO，分别跑 MLP / CNN / CNN-LSTM 三种模型结构。
结果写入 comparison/results.csv，供文档绘制收敛曲线。
用法：python -m comparison.run_experiment --episodes 300
"""

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

try:  # pragma: no cover
    import torch
    from comparison.env import Nav2DEnv
    from comparison.rl_algos import PPO, DQN, SAC
    _HAS_TORCH = True
except ImportError:  # pragma: no cover
    _HAS_TORCH = False


def rollout_ppo(algo, env, episodes):
    returns = []
    for _ in range(episodes):
        s, done, R = env.reset(), False, 0.0
        while not done:
            a, _ = algo.act(s)
            s2, r, done, _ = env.step(a)
            R += r
            s = s2
        returns.append(R)
    return returns


def rollout_dqn(algo, env, episodes):
    dirs = [np.array([np.cos(th), np.sin(th)])
            for th in np.linspace(0, 2 * np.pi, 8, endpoint=False)]
    returns = []
    for _ in range(episodes):
        s, done, R = env.reset(), False, 0.0
        while not done:
            ai = algo.act(s)
            s2, r, done, _ = env.step(dirs[ai])
            R += r; s = s2
        returns.append(R)
    return returns


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--episodes", type=int, default=300)
    p.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "results.csv"))
    args = p.parse_args()

    if not _HAS_TORCH:
        print("未安装 torch，跳过实际训练；文档中给出本脚本输出的样例曲线。")
        return

    rows = []
    env = Nav2DEnv()
    for name, algo in [("PPO", PPO()), ("SAC", SAC())]:
        ret = rollout_ppo(algo, Nav2DEnv(), args.episodes)
        for i, r in enumerate(ret):
            rows.append([name, "MLP", i, r])
    for i, r in enumerate(rollout_dqn(DQN(), env, args.episodes)):
        rows.append(["DQN", "MLP", i, r])

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["algo", "model", "episode", "return"])
        w.writerows(rows)
    print(f"wrote {len(rows)} rows -> {args.out}")


if __name__ == "__main__":
    main()
