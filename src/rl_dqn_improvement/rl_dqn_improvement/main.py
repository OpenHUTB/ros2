"""一键运行 MuJoCo CartPole DQN 改进算法对比全流程。

依次完成：
  1) 训练 3 个对比项（dqn_mlp1、double_mlp1、dueling_mlp1，各 300 回合）
  2) 评估各训练好的模型（确定性策略，20 回合）
  3) 汇总学习曲线（compare_curve.png）与性能对比表（compare_summary.csv）

作为可执行入口：
  - ros2 run rl_dqn_improvement main
  - python3 -m rl_dqn_improvement.main
"""
import argparse
import os

# 默认结果目录：包源码根目录下的 results/（不随当前工作目录漂移）
_DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results')

import sys

from rl_dqn_improvement.compare import main as compare_main
from rl_dqn_improvement.evaluate import main as evaluate_main
from rl_dqn_improvement.train import main as train_main

# 三个对比项：(variant, tag)，固定 MLP(16)、各 300 回合
JOBS = [
    ('dqn', 'dqn_mlp1'),
    ('double', 'double_mlp1'),
    ('dueling', 'dueling_mlp1'),
]


def _run(fn, argv):
    """临时替换 sys.argv 调用子命令 main，用后恢复。"""
    old = sys.argv
    sys.argv = argv
    try:
        fn()
    finally:
        sys.argv = old


def main():
    ap = argparse.ArgumentParser(
        description='一键运行 DQN 改进算法对比全流程')
    ap.add_argument('--episodes', type=int, default=300,
                    help='每个对比项的训练回合数')
    ap.add_argument('--eval-episodes', type=int, default=20,
                    help='评估回合数')
    ap.add_argument('--out', default=(_DEFAULT_OUT if os.path.isdir(_DEFAULT_OUT) else 'results'), help='结果输出目录')
    args = ap.parse_args()

    print('=' * 62)
    print('步骤 1/3：训练 3 个对比项（DQN / Double DQN / Dueling DQN，'
          '各 %d 回合）' % args.episodes)
    print('=' * 62)
    for variant, tag in JOBS:
        _run(train_main,
             ['train.py', '--variant', variant, '--tag', tag,
              '--episodes', str(args.episodes), '--out', args.out])

    print('=' * 62)
    print('步骤 2/3：评估各模型')
    print('=' * 62)
    for _, tag in JOBS:
        _run(evaluate_main,
             ['evaluate.py',
              '--model', os.path.join(args.out, tag + '.pkl'),
              '--episodes', str(args.eval_episodes)])

    print('=' * 62)
    print('步骤 3/3：汇总对比曲线与表格')
    print('=' * 62)
    _run(compare_main, ['compare.py', '--out', args.out])

    print('[main] 全部完成 → %s/*.csv, *.pkl, compare_curve.png, '
          'compare_summary.csv' % args.out)


if __name__ == '__main__':
    main()
