#!/usr/bin/env python3
"""多随机种子基准评测：同一配置重复训练多次，给出均值 ± 标准差。

为什么需要
----------
``train.py`` 只跑一次：CartPole 的初始状态、探索噪声、经验回放采样都是随机的，
单条曲线抖动很大，"DQN 比 REINFORCE 好""两层比一层好"这种结论很容易被一次
运气带偏。基准评测对每组配置跑多个随机种子，用均值 ± 标准差说话。

用法：
  python3 benchmark.py                      # 默认三组配置 × 3 个种子
  python3 benchmark.py --seeds 0,1,2 --episodes 200
  python3 benchmark.py --configs "dqn:16;dqn:16,16:dqn_mlp2;reinforce:16"

输出（默认写到 results/benchmark/）：
  summary.csv   每组配置的均值/标准差/最值
  curves.csv    每个评估点的跨种子均值与标准差
"""
import argparse
import csv
import os
import statistics
import sys

_DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'results', 'benchmark')

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rl_algorithm_compare.train import ALGOS, train_agent  # noqa: E402

#: 默认对比：算法不同 / 网络结构不同，与章节的对比设计一致
DEFAULT_CONFIGS = [
    ('dqn', '16', 'dqn_mlp1'),
    ('dqn', '16,16', 'dqn_mlp2'),
    ('reinforce', '16', 'reinforce_mlp1'),
]


def parse_config(text):
    """解析 ``algo:layers[:tag]``，例如 ``dqn:16,16:dqn_mlp2``。"""
    parts = [p.strip() for p in text.split(':') if p.strip() != '']
    if not parts:
        raise ValueError('配置不能为空')
    algo = parts[0]
    if algo not in ALGOS:
        raise ValueError('未知算法 %r，可选：%s' % (algo, ', '.join(sorted(ALGOS))))
    layers = parts[1] if len(parts) > 1 else '16'
    for item in layers.split(','):
        if int(item) <= 0:
            raise ValueError('隐藏层大小必须为正：%r' % layers)
    if len(parts) > 2:
        tag = parts[2]
    else:
        tag = '%s_mlp%d' % (algo, len(layers.split(',')))
    return algo, layers, tag


def parse_configs(text):
    """分号分隔的多组配置（逗号要留给隐藏层列表，如 ``16,16``）。"""
    if not text:
        return list(DEFAULT_CONFIGS)
    return [parse_config(item) for item in text.split(';') if item.strip()]


def summarize(values):
    """均值 / 样本标准差 / 最小值 / 最大值 / 次数。

    样本标准差（n−1）：种子数通常只有 3~5 个，用总体标准差会低估波动。
    n < 2 时标准差记 0。
    """
    values = [float(v) for v in values]
    if not values:
        raise ValueError('至少需要一个数值')
    mean = statistics.mean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    return {
        'n': len(values),
        'mean': mean,
        'std': std,
        'min': min(values),
        'max': max(values),
    }


def aggregate_curves(curves):
    """把多条学习曲线按评估回合对齐，返回 ``[(episode, mean, std), …]``。

    不同种子的曲线评估点应当一致（都由 ``eval_interval`` 决定）；若某个种子
    少了某个点，则只统计存在的样本。
    """
    buckets = {}
    for curve in curves:
        for episode, value in curve:
            buckets.setdefault(episode, []).append(float(value))
    rows = []
    for episode in sorted(buckets):
        stats = summarize(buckets[episode])
        rows.append((episode, stats['mean'], stats['std']))
    return rows


def run_benchmark(configs, seeds, episodes=120, eval_interval=20,
                  out_dir=None, trainer=None, verbose=True):
    """对每组配置跑 seeds 次，返回结果列表（每项含该配置的统计）。"""
    if not configs:
        raise ValueError('至少需要一组配置')
    if not seeds:
        raise ValueError('至少需要一个随机种子')
    if len(set(seeds)) != len(seeds):
        raise ValueError('随机种子不能重复：%s' % (seeds,))
    trainer = trainer or train_agent

    results = []
    for algo, layers, tag in configs:
        layer_list = [int(x) for x in layers.split(',')]
        runs = []
        for seed in seeds:
            res = trainer(algo, layer_list, episodes,
                          eval_interval=eval_interval, seed=seed, tag=tag)
            runs.append(res)
        finals = [r['final_return'] for r in runs]
        stats = summarize(finals)
        aggregated = aggregate_curves([r['curve'] for r in runs])
        results.append({
            'tag': tag,
            'algo': algo,
            'layers': layers,
            'seeds': list(seeds),
            'final': stats,
            'curves': aggregated,
        })
        if verbose:
            print('  %-16s %-9s mean=%6.1f  std=%5.1f  min=%5.1f  max=%5.1f'
                  % (tag, algo, stats['mean'], stats['std'],
                     stats['min'], stats['max']))

    if out_dir:
        write_outputs(results, out_dir)
    return results


def write_outputs(results, out_dir):
    """写 summary.csv 与 curves.csv。"""
    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    summary_path = os.path.join(out_dir, 'summary.csv')
    with open(summary_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['tag', 'algo', 'layers', 'seeds',
                         'mean_return', 'std_return', 'min_return', 'max_return'])
        for item in results:
            st = item['final']
            writer.writerow([item['tag'], item['algo'], item['layers'],
                             len(item['seeds']), '%.2f' % st['mean'],
                             '%.2f' % st['std'], '%.2f' % st['min'],
                             '%.2f' % st['max']])

    curves_path = os.path.join(out_dir, 'curves.csv')
    with open(curves_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['tag', 'episode', 'mean_return', 'std_return'])
        for item in results:
            for episode, mean, std in item['curves']:
                writer.writerow([item['tag'], episode, '%.2f' % mean, '%.2f' % std])
    return summary_path, curves_path


def format_table(results):
    """把结果排成终端表格。"""
    lines = ['%-16s %-10s %8s %8s %8s %8s'
             % ('配置', '算法', '均值', '标准差', '最小', '最大')]
    lines.append('-' * 62)
    for item in results:
        st = item['final']
        lines.append('%-16s %-10s %8.1f %8.1f %8.1f %8.1f'
                     % (item['tag'], item['algo'], st['mean'], st['std'],
                        st['min'], st['max']))
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description='多随机种子基准评测')
    ap.add_argument('--configs', default='',
                    help='分号分隔的 algo:layers[:tag]；默认三组对比配置')
    ap.add_argument('--seeds', default='0,1,2,3,4', help='随机种子，逗号分隔（建议 >= 5 个）')
    ap.add_argument('--episodes', type=int, default=120)
    ap.add_argument('--eval-interval', type=int, default=20)
    ap.add_argument('--out', default=_DEFAULT_OUT)
    args = ap.parse_args()

    configs = parse_configs(args.configs)
    seeds = [int(x) for x in args.seeds.split(',') if x.strip() != '']
    if not seeds:
        raise SystemExit('至少需要一个种子')

    print('=' * 62)
    print('  CartPole 基准评测：%d 组配置 × %d 个种子 × %d 回合'
          % (len(configs), len(seeds), args.episodes))
    print('=' * 62)
    results = run_benchmark(configs, seeds, episodes=args.episodes,
                            eval_interval=args.eval_interval, out_dir=args.out)
    print()
    print(format_table(results))
    print()
    print('结果已写入: %s' % os.path.abspath(args.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
