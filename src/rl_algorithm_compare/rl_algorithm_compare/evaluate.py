#!/usr/bin/env python3
"""评估已训练的网络模型，统计平均回报与成功率。

用法：
  python3 evaluate.py --model results/dqn_mlp1.pkl --algo dqn --episodes 20
"""
import argparse
import os
import pickle
import sys

import numpy as np

# 支持 `python3 evaluate.py` 直接运行
if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl_algorithm_compare.cartpole_env import CartPoleEnv
from rl_algorithm_compare.dqn import DQN
from rl_algorithm_compare.reinforce import REINFORCE


def load_agent(path, algo):
    """从 pickle 读取网络结构并加载智能体。"""
    with open(path, 'rb') as f:
        d = pickle.load(f)
    sizes = d['sizes']
    layers = sizes[1:-1]
    obs_dim, act_dim = sizes[0], sizes[-1]
    if algo == 'dqn':
        agent = DQN(obs_dim, act_dim, layers)
    else:
        agent = REINFORCE(obs_dim, act_dim, layers)
    return agent.load(path)


def main():
    ap = argparse.ArgumentParser(description='评估 RL 模型')
    ap.add_argument('--model', required=True, help='模型 pkl 路径')
    ap.add_argument('--algo', default='dqn', choices=['dqn', 'reinforce'])
    ap.add_argument('--episodes', type=int, default=20)
    args = ap.parse_args()

    env = CartPoleEnv()
    agent = load_agent(args.model, args.algo)

    rewards = []
    success = 0
    for _ in range(args.episodes):
        s = env.reset()
        done, total = False, 0.0
        while not done:
            a = agent.act(s, greedy=True)
            s, r, done, _ = env.step(a)
            total += r
        rewards.append(total)
        if total >= env.max_steps:  # 撑满一整局=成功保持平衡
            success += 1

    print('[eval] %s episodes=%d mean_return=%.2f std=%.2f success_rate=%.2f'
          % (args.model, args.episodes, float(np.mean(rewards)),
             float(np.std(rewards)), success / args.episodes))


if __name__ == '__main__':
    main()
