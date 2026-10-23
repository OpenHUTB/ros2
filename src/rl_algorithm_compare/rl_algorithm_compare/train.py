#!/usr/bin/env python3
"""MuJoCo CartPole 上训练神经网络 RL 策略并记录学习曲线。

对比维度：
  1) 相同网络结构(MLP 16) 下的不同算法：DQN vs REINFORCE
  2) 相同算法(DQN) 下的不同网络结构：MLP(16) vs MLP(16,16)

用法：
  python3 train.py --algo dqn       --layers 16    --tag dqn_mlp1      --episodes 600
  python3 train.py --algo dqn       --layers 16,16 --tag dqn_mlp2      --episodes 600
  python3 train.py --algo reinforce --layers 16    --tag reinforce_mlp1 --episodes 600
"""
import argparse
import csv
import os

import numpy as np

from .cartpole_env import CartPoleEnv
from .dqn import DQN
from .reinforce import REINFORCE

ALGOS = {'dqn': DQN, 'reinforce': REINFORCE}


def evaluate(agent, env, episodes=10, greedy=True):
    """确定性（greedy）评估若干回合，返回平均回报。"""
    rets = []
    for _ in range(episodes):
        s = env.reset()
        done, ret = False, 0.0
        while not done:
            a = agent.act(s, greedy=greedy)
            s, r, done, _ = env.step(a)
            ret += r
        rets.append(ret)
    return float(np.mean(rets))


def main():
    ap = argparse.ArgumentParser(description='训练 RL 神经网络策略')
    ap.add_argument('--algo', choices=list(ALGOS), default='dqn')
    ap.add_argument('--layers', default='16', help='隐藏层，如 16 或 16,16')
    ap.add_argument('--tag', default='dqn_mlp1')
    ap.add_argument('--episodes', type=int, default=600)
    ap.add_argument('--eval-interval', type=int, default=30)
    ap.add_argument('--out', default='results')
    ap.add_argument('--render-gif', action='store_true')
    ap.add_argument('--gif-episodes', type=int, default=1)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    layers = [int(x) for x in args.layers.split(',')]
    env = CartPoleEnv()
    agent = ALGOS[args.algo](env.observation_dim, env.action_dim, layers)

    csv_path = os.path.join(args.out, f'{args.tag}.csv')
    with open(csv_path, 'w', newline='') as f:
        csv.writer(f).writerow(['episode', 'mean_return'])

    print('[train] %s layers=%s tag=%s episodes=%d'
          % (args.algo, args.layers, args.tag, args.episodes))
    for ep in range(1, args.episodes + 1):
        s = env.reset()
        done, ret, traj = False, 0.0, []
        while not done:
            a = agent.act(s)
            s2, r, done, _ = env.step(a)
            ret += r
            if args.algo == 'dqn':
                agent.store(s, a, r, s2, done)
                agent.update()
            else:
                traj.append((s, a, r))
            s = s2
        if args.algo == 'reinforce':
            agent.update(traj)
        if ep % args.eval_interval == 0:
            m = evaluate(agent, env)
            with open(csv_path, 'a', newline='') as f:
                csv.writer(f).writerow([ep, round(m, 2)])
            print('  ep=%d mean_return=%.1f' % (ep, m))

    model_path = os.path.join(args.out, f'{args.tag}.pkl')
    agent.save(model_path)
    print('[train] 模型已保存: %s' % model_path)

    if args.render_gif:
        from . import render_gif
        render_gif.render(agent, env,
                          os.path.join(args.out, f'{args.tag}.gif'),
                          episodes=args.gif_episodes)


if __name__ == '__main__':
    main()
