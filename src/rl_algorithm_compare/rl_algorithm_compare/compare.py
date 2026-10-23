#!/usr/bin/env python3
"""汇总多个训练 csv，生成学习曲线对比图与对比表。

读取 results/*.csv（每行: episode, mean_return），绘制学习曲线并输出
compare_curve.png 与 compare_summary.csv。

用法：
  python3 compare.py --out results
  python3 compare.py --out results --tags dqn_mlp1 dqn_mlp2 reinforce_mlp1
"""
import argparse
import csv
import glob
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser(description='汇总对比结果')
    ap.add_argument('--out', default='results')
    ap.add_argument('--tags', nargs='*', default=None)
    args = ap.parse_args()

    if args.tags:
        csv_files = [os.path.join(args.out, t + '.csv') for t in args.tags]
    else:
        csv_files = sorted(glob.glob(os.path.join(args.out, '*.csv')))

    if not csv_files:
        print('[compare] 没有找到 csv，请先运行 train.py')
        return

    fig, ax = plt.subplots(figsize=(8, 5))
    summary = []
    for f in csv_files:
        tag = os.path.splitext(os.path.basename(f))[0]
        ep, ret = [], []
        with open(f, newline='') as fh:
            next(fh)  # header
            for row in csv.reader(fh):
                if row:
                    ep.append(int(row[0]))
                    ret.append(float(row[1]))
        if not ep:
            continue
        ax.plot(ep, ret, marker='.', linewidth=1.5, label=tag)
        tail = ret[-min(10, len(ret)):]
        summary.append([tag, ep[-1], float(max(ret)), float(np.mean(tail))])

    ax.set_xlabel('Training Episode')
    ax.set_ylabel('Mean Return (eval)')
    ax.set_title('RL Algorithms / Network Structures Comparison (MuJoCo CartPole)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'compare_curve.png'), dpi=120)
    print('[compare] 已保存 compare_curve.png')

    summary_path = os.path.join(args.out, 'compare_summary.csv')
    with open(summary_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['tag', 'final_episode', 'max_return', 'final_mean_return'])
        w.writerows(summary)
    print('[compare] 已保存 compare_summary.csv')
    for s in summary:
        print('  ', s)


if __name__ == '__main__':
    main()
