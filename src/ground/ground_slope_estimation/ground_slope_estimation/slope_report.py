"""汇总坡度估计结果并生成对比图。

用法:
    ros2 run ground_slope_estimation slope_report --results results
"""

import argparse
import csv
import glob
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'WenQuanYi Zen Hei',
                                   'AR PL UMing CN', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

COLUMNS = ('slope_true', 'slope_ls', 'slope_ransac', 'noise_std',
           'n_points', 'height_ls', 'ransac_inlier_ratio', 'inlier_ratio_ls')


def load_runs(results_dir):
    """读取目录下全部 scan_*.csv，返回按 (坡度, 噪声) 排序的行列表。"""
    rows = []
    for path in sorted(glob.glob(os.path.join(results_dir, 'scan_*.csv'))):
        with open(path) as handle:
            reader = csv.DictReader(handle)
            for record in reader:
                try:
                    rows.append({key: float(record[key]) for key in COLUMNS})
                except (KeyError, TypeError, ValueError):
                    raise SystemExit(
                        '{} 内容不完整或格式错误，请删除后重跑'.format(path))
    if not rows:
        raise SystemExit('目录 {} 中没有 scan_*.csv'.format(results_dir))
    rows.sort(key=lambda item: (item['slope_true'], item['noise_std']))
    return rows


def print_table(rows):
    print('| 真值(°) | 噪声(mm) | 最小二乘(°) | RANSAC(°) | '
          'LS 误差(°) | RANSAC 误差(°) | 高程误差(m) |')
    print('| --- | --- | --- | --- | --- | --- | --- |')
    for row in rows:
        print('| {:.1f} | {:.1f} | {:.3f} | {:.3f} | {:+.3f} | {:+.3f} | '
              '{:+.4f} |'.format(
                  row['slope_true'], row['noise_std'] * 1000.0,
                  row['slope_ls'], row['slope_ransac'],
                  row['slope_ls'] - row['slope_true'],
                  row['slope_ransac'] - row['slope_true'],
                  row['height_ls']))


def save_figure(rows, figure_path):
    slopes = sorted({row['slope_true'] for row in rows})
    noises = sorted({row['noise_std'] for row in rows})
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))

    ax = axes[0]
    true_values = [row['slope_true'] for row in rows]
    limit = max(true_values) * 1.15 or 1.0
    ax.plot([0, limit], [0, limit], color='0.5', linestyle='--',
            linewidth=0.9, label='理想估计')
    ax.scatter(true_values, [row['slope_ls'] for row in rows],
               s=36, color='#1f77b4', label='最小二乘')
    ax.scatter(true_values, [row['slope_ransac'] for row in rows],
               s=36, color='#d62728', marker='^', label='RANSAC')
    ax.set_xlabel('真实坡度 (°)')
    ax.set_ylabel('估计坡度 (°)')
    ax.set_title('估计值与真值')
    ax.legend()
    ax.grid(alpha=0.3)

    ax = axes[1]
    colors = plt.cm.viridis(np.linspace(0.15, 0.85, len(slopes)))
    for slope, color in zip(slopes, colors):
        subset = [row for row in rows if row['slope_true'] == slope]
        ax.plot([row['noise_std'] * 1000.0 for row in subset],
                [row['slope_ls'] - row['slope_true'] for row in subset],
                marker='o', color=color, label='{:.0f}°'.format(slope))
    ax.axhline(0.0, color='0.5', linewidth=0.9)
    ax.set_xlabel('深度噪声标准差 (mm)')
    ax.set_ylabel('最小二乘坡度误差 (°)')
    ax.set_title('噪声对精度的影响')
    ax.legend(title='坡度')
    ax.grid(alpha=0.3)

    ax = axes[2]
    width = 0.8 / max(len(noises), 1)
    for index, noise in enumerate(noises):
        subset = [row for row in rows if row['noise_std'] == noise]
        offsets = np.arange(len(subset)) + (index - len(noises) / 2.0) * width
        ax.bar(offsets, [row['height_ls'] * 1000.0 for row in subset],
               width=width * 0.9,
               label='{:.0f} mm'.format(noise * 1000.0))
    ax.set_xticks(np.arange(len(slopes)))
    ax.set_xticklabels(['{:.0f}°'.format(slope) for slope in slopes])
    ax.set_xlabel('真实坡度')
    ax.set_ylabel('原点处高程误差 (mm)')
    ax.set_title('绝对高程偏差')
    ax.legend(title='噪声')
    ax.grid(alpha=0.3, axis='y')

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(figure_path)), exist_ok=True)
    fig.savefig(figure_path, dpi=150)
    plt.close(fig)


def main(args=None):
    parser = argparse.ArgumentParser(description='地面坡度估计结果汇总')
    parser.add_argument('--results', default='results')
    parser.add_argument('--figure', default='results/effect.png')
    parsed = args if isinstance(args, argparse.Namespace) \
        else parser.parse_args(args)

    rows = load_runs(parsed.results)
    print_table(rows)

    os.makedirs(parsed.results, exist_ok=True)
    with open(os.path.join(parsed.results, 'summary.csv'), 'w',
              newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    save_figure(rows, parsed.figure)
    print('图已保存到 {}'.format(parsed.figure))


if __name__ == '__main__':
    main()
