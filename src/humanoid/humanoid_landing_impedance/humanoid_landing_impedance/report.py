"""汇总多次落地实验的 CSV，计算指标并生成对比图。

用法:
    ros2 run humanoid_landing_impedance landing_report \
        --results results --figure results/effect.png
"""

import argparse
import csv
import glob
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Linux 服务器上中文字体常为 Noto Sans CJK，按优先级回退避免乱码方块
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'WenQuanYi Zen Hei',
                                   'AR PL UMing CN', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

from .metrics import compute_metrics

ROBOT_MASS = 3.0
MODE_LABELS = {'stiff': '刚性保持', 'impedance': '阻抗缓冲'}
MODE_COLORS = {'stiff': '#d62728', 'impedance': '#1f77b4'}
SAMPLE_FILE = re.compile(r'samples_([a-z]+)_(\d+\.\d+)\.csv$')


def load_runs(results_dir):
    """读取目录下全部 samples_<mode>_<drop>.csv，返回 (mode, drop) 字典。"""
    runs = {}
    for path in sorted(glob.glob(os.path.join(results_dir, 'samples_*.csv'))):
        match = SAMPLE_FILE.search(os.path.basename(path))
        if not match:
            continue
        mode, drop = match.group(1), float(match.group(2))
        with open(path) as handle:
            rows = [{key: float(value) for key, value in row.items()}
                    for row in csv.DictReader(handle)]
        runs[(mode, drop)] = rows
    return runs


def summarize(runs):
    """对每个 (mode, drop) 组合计算指标，返回列表并按 (drop, mode) 排序。"""
    table = []
    for (mode, drop), samples in runs.items():
        metrics = compute_metrics(samples, ROBOT_MASS)
        metrics['mode'] = mode
        metrics['drop'] = drop
        table.append(metrics)
    table.sort(key=lambda item: (item['drop'], item['mode']))
    return table


def print_table(table):
    """以 Markdown 表格打印指标汇总。"""
    header = ('| 跌落高度(m) | 控制器 | 首峰GRF(N) | 主动峰GRF(N) | '
              '冲量(N·s) | 最大屈膝(rad) | 最大俯仰(rad) | 恢复时间(s) |')
    print(header)
    print('| --- | --- | --- | --- | --- | --- | --- | --- |')
    for row in table:
        def fmt(value):
            return '—' if value is None else '{:.2f}'.format(value)
        print('| {:.2f} | {} | {} | {} | {} | {} | {} | {} |'.format(
            row['drop'], MODE_LABELS.get(row['mode'], row['mode']),
            fmt(row['first_peak_grf']), fmt(row['active_peak_grf']),
            fmt(row['loading_impulse']), fmt(row['max_knee_flexion']),
            fmt(row['max_pitch']), fmt(row['settle_time'])))


def _pick_curve_drops(drops):
    """选居中的跌落高度画 GRF 曲线。"""
    return sorted(drops)[len(drops) // 2] if drops else None


def save_figure(runs, table, figure_path):
    """三联图: GRF 曲线、主动峰柱状图、屈膝行程柱状图。"""
    drops = sorted({row['drop'] for row in table})
    modes = [mode for mode in ('stiff', 'impedance')
             if any(row['mode'] == mode for row in table)]
    curve_drop = _pick_curve_drops(drops)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))

    ax = axes[0]
    for mode in modes:
        samples = runs.get((mode, curve_drop))
        if not samples:
            continue
        ax.plot([s['t'] for s in samples], [s['grf'] for s in samples],
                color=MODE_COLORS[mode], linewidth=1.2,
                label=MODE_LABELS.get(mode, mode))
    ax.set_title('地面反力曲线（跌落 {:.2f} m）'.format(curve_drop or 0.0))
    ax.set_xlabel('仿真时间 (s)')
    ax.set_ylabel('地面反力 (N)')
    ax.legend()
    ax.grid(alpha=0.3)

    width = 0.35
    for col, key, title in (
            (1, 'active_peak_grf', '主动峰地面反力'),
            (2, 'max_knee_flexion', '最大膝关节屈曲')):
        ax = axes[col]
        for index, mode in enumerate(modes):
            values = []
            for drop in drops:
                row = next((r for r in table
                            if r['mode'] == mode and r['drop'] == drop), None)
                values.append(row[key] if row and row[key] is not None
                              else float('nan'))
            offsets = [i + (index - (len(modes) - 1) / 2.0) * width
                       for i in range(len(drops))]
            ax.bar(offsets, values, width=width * 0.9,
                   color=MODE_COLORS[mode],
                   label=MODE_LABELS.get(mode, mode))
        ax.set_xticks(range(len(drops)))
        ax.set_xticklabels(['{:.2f} m'.format(d) for d in drops])
        ax.set_title(title)
        ax.grid(alpha=0.3, axis='y')
        if col == 1:
            ax.set_ylabel('力 (N)')
            ax.legend()
        else:
            ax.set_ylabel('关节角 (rad)')

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(figure_path)),
                exist_ok=True)
    fig.savefig(figure_path, dpi=150)
    plt.close(fig)


def main(args=None):
    parser = argparse.ArgumentParser(description='落地缓冲实验指标汇总')
    parser.add_argument('--results', default='results', help='CSV 目录')
    parser.add_argument('--figure', default='results/effect.png',
                        help='对比图输出路径')
    parsed = args if isinstance(args, argparse.Namespace) \
        else parser.parse_args(args)

    runs = load_runs(parsed.results)
    if not runs:
        raise SystemExit('目录 {} 中没有 samples_*.csv'.format(parsed.results))
    table = summarize(runs)
    print_table(table)

    metrics_path = os.path.join(parsed.results, 'metrics.csv')
    fieldnames = ('mode', 'drop', 'touchdown', 'first_peak_grf',
                  'active_peak_grf', 'loading_impulse',
                  'max_knee_flexion', 'max_pitch', 'settle_time')
    with open(metrics_path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in table:
            writer.writerow({key: row.get(key) for key in fieldnames})
    print('指标已写入 {}'.format(metrics_path))

    save_figure(runs, table, parsed.figure)
    print('对比图已保存到 {}'.format(parsed.figure))


if __name__ == '__main__':
    main()
