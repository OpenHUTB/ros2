"""汇总多组 (kp, kd) 阶跃实验，计算指标并生成参数整定图。

用法:
    ros2 run humanoid_knee_impedance knee_report \
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

from .metrics import (compute_metrics, damping_ratio, natural_frequency,
                      predicted_steady_state_error)

SAMPLE_FILE = re.compile(r'samples_kp([\d.]+)_kd([\d.]+)\.csv$')
SERIES_COLORS = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd', '#ff7f0e']


def load_runs(results_dir):
    """读取目录下全部 samples_kp*_kd*.csv，返回 (kp, kd) 字典。"""
    runs = {}
    for path in sorted(glob.glob(os.path.join(results_dir,
                                              'samples_kp*_kd*.csv'))):
        match = SAMPLE_FILE.search(os.path.basename(path))
        if not match:
            continue
        kp, kd = float(match.group(1)), float(match.group(2))
        with open(path) as handle:
            try:
                rows = [{key: float(value) for key, value in row.items()}
                        for row in csv.DictReader(handle)]
            except (TypeError, ValueError):
                raise SystemExit(
                    '{} 内容不完整或格式错误，请删除后重新运行实验'.format(path))
        if not rows:
            raise SystemExit('{} 中没有采样数据'.format(path))
        runs[(kp, kd)] = rows
    return runs


def summarize(runs):
    """对每个 (kp, kd) 组合计算指标，按 (kp, kd) 排序。"""
    table = []
    for (kp, kd), samples in runs.items():
        command = samples[-1]['q_cmd']
        metrics = compute_metrics(samples, command)
        metrics['kp'] = kp
        metrics['kd'] = kd
        metrics['q_cmd'] = command
        metrics['omega_n'] = natural_frequency(kp)
        metrics['zeta'] = damping_ratio(kp, kd)
        table.append(metrics)
    table.sort(key=lambda item: (item['kp'], item['kd']))
    return table


def _fmt(value, digits=3):
    return '—' if value is None else ('{:.' + str(digits) + 'f}').format(value)


def print_table(table):
    """以 Markdown 表格打印指标汇总。"""
    print('| kp(N·m/rad) | kd(N·m·s/rad) | ζ | ω_n(rad/s) | 上升时间(s) | '
          '超调(%) | 调节时间(s) | 稳态误差(rad) | 峰值力矩(N·m) |')
    print('| --- | --- | --- | --- | --- | --- | --- | --- | --- |')
    for row in table:
        print('| {:.2f} | {:.2f} | {:.2f} | {:.1f} | {} | {} | {} | {} | {} |'
              .format(row['kp'], row['kd'], row['zeta'], row['omega_n'],
                      _fmt(row['rise_time']),
                      _fmt(row['overshoot'], 1),
                      _fmt(row['settling_time']),
                      _fmt(row['steady_state_error']),
                      _fmt(row['peak_torque'], 2)))


def _grid(table):
    """返回 (kp 列表, kd 列表)。"""
    return (sorted({row['kp'] for row in table}),
            sorted({row['kd'] for row in table}))


def _lookup(table, kp, kd):
    return next((row for row in table
                 if row['kp'] == kp and row['kd'] == kd), None)


def save_figure(runs, table, figure_path):
    """四联图: 阶跃响应、上升时间、超调、稳态误差。"""
    kps, kds = _grid(table)
    fig, axes = plt.subplots(1, 4, figsize=(19, 4.2))

    # 1) 阶跃响应曲线：取居中的 kp，画全部 kd
    ax = axes[0]
    kp_mid = kps[len(kps) // 2]
    for index, kd in enumerate(kds):
        samples = runs.get((kp_mid, kd))
        if not samples:
            continue
        ax.plot([s['t'] for s in samples], [s['q'] for s in samples],
                color=SERIES_COLORS[index % len(SERIES_COLORS)],
                linewidth=1.3, label='kd = {:.2f}'.format(kd))
    ax.axhline(table[0]['q_cmd'], color='0.4', linestyle='--',
               linewidth=0.9, label='目标角')
    ax.set_xlim(0.0, 3.0)
    ax.set_title('膝关节阶跃响应（kp = {:.1f}）'.format(kp_mid))
    ax.set_xlabel('阶跃后时间 (s)')
    ax.set_ylabel('膝关节角 (rad)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # 2) 上升时间 vs kp，每条线对应一个 kd
    ax = axes[1]
    for index, kd in enumerate(kds):
        values = [_lookup(table, kp, kd)['rise_time'] for kp in kps]
        ax.plot(kps, [float('nan') if v is None else v for v in values],
                marker='o', color=SERIES_COLORS[index % len(SERIES_COLORS)],
                label='kd = {:.2f}'.format(kd))
    ax.set_title('上升时间（10%→90%）')
    ax.set_xlabel('kp (N·m/rad)')
    ax.set_ylabel('上升时间 (s)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # 3) 超调 vs kd，每条线对应一个 kp
    ax = axes[2]
    for index, kp in enumerate(kps):
        values = [_lookup(table, kp, kd)['overshoot'] for kd in kds]
        ax.plot(kds, values, marker='s',
                color=SERIES_COLORS[index % len(SERIES_COLORS)],
                label='kp = {:.1f}'.format(kp))
    ax.set_title('超调量')
    ax.set_xlabel('kd (N·m·s/rad)')
    ax.set_ylabel('超调 (%)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # 4) 稳态误差 vs kp，附 mgl·sin(q_cmd)/kp 参考线
    ax = axes[3]
    width = 0.8 / max(1, len(kds))
    for index, kd in enumerate(kds):
        values = [_lookup(table, kp, kd)['steady_state_error'] for kp in kps]
        offsets = [i + (index - (len(kds) - 1) / 2.0) * width
                   for i in range(len(kps))]
        ax.bar(offsets, values, width=width * 0.9,
               color=SERIES_COLORS[index % len(SERIES_COLORS)],
               label='kd = {:.2f}'.format(kd))
    command = table[0]['q_cmd']
    reference = [predicted_steady_state_error(kp, command) for kp in kps]
    ax.plot(range(len(kps)), reference, 'k--', linewidth=1.0,
            label='理论 mgl·sin(q)/kp')
    ax.set_xticks(range(len(kps)))
    ax.set_xticklabels(['{:.1f}'.format(kp) for kp in kps])
    ax.set_title('稳态误差（重力负载）')
    ax.set_xlabel('kp (N·m/rad)')
    ax.set_ylabel('稳态误差 (rad)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis='y')

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(figure_path)), exist_ok=True)
    fig.savefig(figure_path, dpi=150)
    plt.close(fig)


def main(args=None):
    parser = argparse.ArgumentParser(description='膝关节阻抗参数整定汇总')
    parser.add_argument('--results', default='results', help='CSV 目录')
    parser.add_argument('--figure', default='results/effect.png',
                        help='对比图输出路径')
    parsed = args if isinstance(args, argparse.Namespace) \
        else parser.parse_args(args)

    runs = load_runs(parsed.results)
    if not runs:
        raise SystemExit('目录 {} 中没有 samples_kp*_kd*.csv'
                         .format(parsed.results))
    table = summarize(runs)
    print_table(table)

    metrics_path = os.path.join(parsed.results, 'metrics.csv')
    fieldnames = ('kp', 'kd', 'zeta', 'omega_n', 'rise_time', 'overshoot',
                  'settling_time', 'steady_state_error', 'peak_torque',
                  'peak_velocity', 'final_value', 'q_cmd')
    with open(metrics_path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in table:
            writer.writerow({key: row.get(key) for key in fieldnames})
    print('指标已写入 {}'.format(metrics_path))

    save_figure(runs, table, parsed.figure)
    print('整定图已保存到 {}'.format(parsed.figure))


if __name__ == '__main__':
    main()
