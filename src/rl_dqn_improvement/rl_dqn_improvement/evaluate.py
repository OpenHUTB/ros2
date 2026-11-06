#!/usr/bin/env python3
"""评估训练好的 DQN 变体模型在 MuJoCo CartPole 上的性能。

用法：
  python3 -m rl_dqn_improvement.evaluate --model results/dqn_mlp1.pkl
"""
import argparse
import os

import numpy as np

# 支持 `python3 xxx.py` 直接运行：无包上下文时把仓库 src/ 加入 sys.path
if __package__ in (None, ''):
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl_dqn_improvement.cartpole_env import CartPoleEnv


def main():
    ap = argparse.ArgumentParser(description='评估 DQN / Double / Dueling 模型')
    ap.add_argument('--model', required=True, help='模型 pkl 路径')
    ap.add_argument('--episodes', type=int, default=20)
    args = ap.parse_args()

    env = CartPoleEnv()
    agent = DQN(env.observation_dim, env.action_dim, [16]).load(args.model)
    rets = []
    for _ in range(args.episodes):
        s = env.reset()
        done, ret = False, 0.0
        while not done:
            a = agent.act(s, greedy=True)
            s, r, done, _ = env.step(a)
            ret += r
        rets.append(ret)
    rets = np.array(rets)
    print('[eval] %s episodes=%d mean_return=%.2f std=%.2f success_rate=%.2f'
          % (args.model, args.episodes, rets.mean(), rets.std(),
             float(np.mean(rets >= env.max_steps))))


if __name__ == '__main__':
    main()
