"""一键运行 MuJoCo CartPole RL 算法 / 网络结构对比全流程。

依次完成：
  1) 训练 3 个对比项（DQN-MLP16、DQN-MLP(16,16)、REINFORCE-MLP16）
  2) 评估各训练好的模型（平均回报）
  3) 汇总学习曲线（compare_curve.png）与性能对比表（compare_summary.csv）

作为可执行入口：
  - ros2 run rl_algorithm_compare main
  - python3 -m rl_algorithm_compare.main
  - python3 main.py            （仓库 src/ 下直接运行）
"""
import argparse
import os

# 默认结果目录：包源码根目录下的 results/（不随当前工作目录漂移）
_DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results')

import sys

from rl_algorithm_compare.compare import main as compare_main
from rl_algorithm_compare.evaluate import main as evaluate_main
from rl_algorithm_compare.train import main as train_main

# 三个对比项：(算法, 隐藏层, 结果标签)
JOBS = [
    ('dqn', '16', 'dqn_mlp1', 300),
    ('dqn', '16,16', 'dqn_mlp2', 300),
    ('reinforce', '16', 'reinforce_mlp1', 600),  # 收敛较慢，默认多训
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
        description='一键运行 RL 算法与网络结构对比全流程')
    ap.add_argument('--episodes', type=int, default=None,
                    help='训练回合数；不指定则按对比项默认（DQN 300，'
                         'REINFORCE 600）')
    ap.add_argument('--eval-episodes', type=int, default=20,
                    help='评估回合数')
    ap.add_argument('--out', default=(_DEFAULT_OUT if os.path.isdir(_DEFAULT_OUT) else 'results'), help='结果输出目录')
    args = ap.parse_args()

    print('=' * 62)
    print('步骤 1/3：训练 3 个对比项')
    print('  DQN-MLP16 / DQN-MLP(16,16) / REINFORCE-MLP16'
          + ('，各 %d 回合' % args.episodes if args.episodes
             else '（默认回合数：DQN 300，REINFORCE 600）'))
    print('=' * 62)
    for algo, layers, tag, default_ep in JOBS:
        ep = args.episodes if args.episodes is not None else default_ep
        _run(train_main,
             ['train.py', '--algo', algo, '--layers', layers, '--tag', tag,
              '--episodes', str(ep), '--out', args.out])

    print('=' * 62)
    print('步骤 2/3：评估各模型')
    print('=' * 62)
    for algo, _, tag in JOBS:
        _run(evaluate_main,
             ['evaluate.py',
              '--model', os.path.join(args.out, tag + '.pkl'),
              '--algo', algo, '--episodes', str(args.eval_episodes)])

    print('=' * 62)
    print('步骤 3/3：汇总对比曲线与表格')
    print('=' * 62)
    _run(compare_main, ['compare.py', '--out', args.out])

    print('[main] 全部完成 → %s/*.csv, *.pkl, compare_curve.png, '
          'compare_summary.csv' % args.out)


if __name__ == '__main__':
    main()
