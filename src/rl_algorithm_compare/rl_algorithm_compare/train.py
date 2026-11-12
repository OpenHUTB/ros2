#!/usr/bin/env python3
"""MuJoCo CartPole 上训练神经网络 RL 策略并记录学习曲线。

对比维度：
  1) 相同网络结构(MLP 16) 下的不同算法：DQN vs REINFORCE
  2) 相同算法(DQN) 下的不同网络结构：MLP(16) vs MLP(16,16)

用法：
  python3 train.py --algo dqn       --layers 16    --tag dqn_mlp1      --episodes 600
  python3 train.py --algo dqn       --layers 16,16 --tag dqn_mlp2      --episodes 600
  python3 train.py --algo reinforce --layers 16    --tag reinforce_mlp1 --episodes 600

也提供 ``train_agent()`` 供 ``benchmark.py`` 复用：给定随机种子训练一次，
返回学习曲线与最终回报。
"""
import argparse
import csv
import os

# 默认结果目录：包源码根目录下的 results/（不随当前工作目录漂移）
_DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results')

import sys

import numpy as np

# 支持 `python3 train.py` 直接运行：无包上下文时把仓库 src/ 加入 sys.path
if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl_algorithm_compare.dqn import DQN
from rl_algorithm_compare.reinforce import REINFORCE

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


def train_agent(algo, layers, episodes, eval_interval=30, seed=None,
                out_dir=None, tag=None, env=None, verbose=True,
                render_gif=False, gif_episodes=1):
    """训练一个智能体并返回结果字典。

    参数
    ----
    seed        随机种子；设了就固定 ``numpy.random`` 的全局状态，
                保证同一配置重复跑得到同样的曲线（基准评测要用）
    out_dir     非空时把学习曲线写成 ``<tag>.csv`` 并保存模型 ``<tag>.pkl``
    env         可注入已有的环境对象（默认新建）

    返回 dict：
        algo / layers / episodes / seed / tag
        curve        [(episode, mean_return), …]
        final_return 最后一次评估的平均回报
        agent        训练好的智能体（可直接 evaluate / save）
        csv_path / model_path   输出路径（out_dir 为空时为 None）
    """
    if seed is not None:
        np.random.seed(seed)
    if env is None:
        # 延迟导入：让 benchmark / 单元测试在没装 mujoco 的机器上也能 import 本模块
        from rl_algorithm_compare.cartpole_env import CartPoleEnv
        env = CartPoleEnv()
    # seed 同时管三件事：环境初值/探索（全局 np.random）、回放采样、网络初始化
    agent = ALGOS[algo](env.observation_dim, env.action_dim, layers, seed=seed)

    tag = tag or algo
    csv_path = None
    model_path = None
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        csv_path = os.path.join(out_dir, '%s.csv' % tag)
        with open(csv_path, 'w', newline='') as f:
            csv.writer(f).writerow(['episode', 'mean_return'])

    if verbose:
        print('[train] %s layers=%s tag=%s episodes=%d%s'
              % (algo, layers, tag, episodes,
                 '' if seed is None else ' seed=%d' % seed))

    curve = []
    for ep in range(1, episodes + 1):
        s = env.reset()
        done, ret, traj = False, 0.0, []
        while not done:
            a = agent.act(s)
            s2, r, done, _ = env.step(a)
            ret += r
            if algo == 'dqn':
                agent.store(s, a, r, s2, done)
                agent.update()
            else:
                traj.append((s, a, r))
            s = s2
        if algo == 'reinforce':
            agent.update(traj)
        if ep % eval_interval == 0:
            m = evaluate(agent, env)
            curve.append((ep, m))
            if csv_path:
                with open(csv_path, 'a', newline='') as f:
                    csv.writer(f).writerow([ep, round(m, 2)])
            if verbose:
                print('  ep=%d mean_return=%.1f' % (ep, m))

    if out_dir:
        model_path = os.path.join(out_dir, '%s.pkl' % tag)
        agent.save(model_path)
        if verbose:
            print('[train] 模型已保存: %s' % model_path)

    if render_gif and out_dir:
        from rl_algorithm_compare import render_gif
        render_gif.render(agent, env,
                          os.path.join(out_dir, '%s.gif' % tag),
                          episodes=gif_episodes)

    final_return = curve[-1][1] if curve else evaluate(agent, env)
    return {
        'algo': algo,
        'layers': list(layers),
        'episodes': episodes,
        'seed': seed,
        'curve': curve,
        'final_return': final_return,
        'agent': agent,
        'csv_path': csv_path,
        'model_path': model_path,
        'tag': tag,
    }


def main():
    ap = argparse.ArgumentParser(description='训练 RL 神经网络策略')
    ap.add_argument('--algo', choices=list(ALGOS), default='dqn')
    ap.add_argument('--layers', default='16', help='隐藏层，如 16 或 16,16')
    ap.add_argument('--tag', default='dqn_mlp1')
    ap.add_argument('--episodes', type=int, default=600)
    ap.add_argument('--eval-interval', type=int, default=30)
    ap.add_argument('--out', default=(_DEFAULT_OUT if os.path.isdir(_DEFAULT_OUT) else 'results'))
    ap.add_argument('--seed', type=int, default=None, help='随机种子（可复现）')
    ap.add_argument('--render-gif', action='store_true')
    ap.add_argument('--gif-episodes', type=int, default=1)
    args = ap.parse_args()

    layers = [int(x) for x in args.layers.split(',')]
    train_agent(args.algo, layers, args.episodes,
                eval_interval=args.eval_interval, seed=args.seed,
                out_dir=args.out, tag=args.tag,
                render_gif=args.render_gif, gif_episodes=args.gif_episodes)


if __name__ == '__main__':
    main()
