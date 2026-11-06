#!/usr/bin/env python3
"""在 MuJoCo CartPole 上训练 DQN 及其改进算法（Double / Dueling）并记录学习曲线。

对比维度：相同环境、相同网络结构（MLP16）下的 DQN 算法改进对比：
  - dqn    ：标准 DQN（基线）
  - double ：Double DQN（在线网选动作 + 目标网评估）
  - dueling：Dueling DQN（状态价值 + 优势 双流网络）

用法：
  python3 -m rl_dqn_improvement.train --variant dqn     --tag dqn_mlp1
  python3 -m rl_dqn_improvement.train --variant double  --tag double_mlp1
  python3 -m rl_dqn_improvement.train --variant dueling --tag dueling_mlp1
"""
import argparse
import csv
import os

# 默认结果目录：包源码根目录下的 results/（不随当前工作目录漂移）
_DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results')


import numpy as np

# 支持 `python3 xxx.py` 直接运行：无包上下文时把仓库 src/ 加入 sys.path
if __package__ in (None, ''):
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl_dqn_improvement.cartpole_env import CartPoleEnv

VARIANTS = ('dqn', 'double', 'dueling')


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
    ap = argparse.ArgumentParser(description='训练 DQN / Double / Dueling 神经网络策略')
    ap.add_argument('--variant', choices=VARIANTS, default='dqn',
                    help='DQN 变体: dqn/double/dueling')
    ap.add_argument('--layers', default='16', help='隐藏层，如 16 或 16,16')
    ap.add_argument('--tag', default='dqn_mlp1')
    ap.add_argument('--episodes', type=int, default=300)
    ap.add_argument('--eval-interval', type=int, default=30)
    ap.add_argument('--out', default=(_DEFAULT_OUT if os.path.isdir(_DEFAULT_OUT) else 'results'))
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    layers = [int(x) for x in args.layers.split(',')]
    env = CartPoleEnv()
    agent = DQN(env.observation_dim, env.action_dim, layers, variant=args.variant)

    csv_path = os.path.join(args.out, f'{args.tag}.csv')
    with open(csv_path, 'w', newline='') as f:
        csv.writer(f).writerow(['episode', 'mean_return'])

    print('[train] variant=%s layers=%s tag=%s episodes=%d'
          % (args.variant, args.layers, args.tag, args.episodes))
    for ep in range(1, args.episodes + 1):
        s = env.reset()
        done, ret = False, 0.0
        while not done:
            a = agent.act(s)
            s2, r, done, _ = env.step(a)
            ret += r
            agent.store(s, a, r, s2, done)
            agent.update()
            s = s2
        if ep % args.eval_interval == 0:
            m = evaluate(agent, env)
            with open(csv_path, 'a', newline='') as f:
                csv.writer(f).writerow([ep, round(m, 2)])
            print('  ep=%d mean_return=%.1f' % (ep, m))

    model_path = os.path.join(args.out, f'{args.tag}.pkl')
    agent.save(model_path)
    print('[train] 模型已保存: %s' % model_path)


if __name__ == '__main__':
    main()
