"""离线评估：把估计姿态与真值按时间对齐，统计误差并绘制对比图。

用法（在包目录下）：
  ros2 run humanoid_imu_attitude attitude_eval --results results --figure results/effect.png
  ros2 run humanoid_imu_attitude attitude_eval --bench
"""

import argparse
import csv
import math
import os
import time

import numpy as np

from humanoid_imu_attitude.quat_math import (
    complementary_step,
    gravity_in_body,
    madgwick_step,
    quat_from_euler,
)

PROFILES = ('static', 'slow', 'fast')
METHODS = ('complementary', 'madgwick')
COLORS = {
    'truth': '#333333',
    'complementary': '#1f77b4',
    'madgwick': '#d62728',
}


def load_series(path):
    with open(path, 'r') as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r['t']) for r in rows])
    roll = np.array([float(r['roll_deg']) for r in rows])
    pitch = np.array([float(r['pitch_deg']) for r in rows])
    yaw = np.array([float(r['yaw_deg']) for r in rows])
    return t, np.column_stack([roll, pitch, yaw])


def align_and_error(gt_t, gt_deg, est_t, est_deg):
    """把真值插值到估计时刻，返回 (t, 参考值, 误差)，误差折算到 [-180, 180]。"""
    mask = (est_t >= gt_t[0]) & (est_t <= gt_t[-1])
    est_t = est_t[mask]
    est_deg = est_deg[mask]
    ref = np.column_stack([
        np.interp(est_t, gt_t, gt_deg[:, index]) for index in range(est_deg.shape[1])
    ])
    err = (est_deg - ref + 180.0) % 360.0 - 180.0
    return est_t, ref, err


def compute_metrics(gt_t, gt_deg, est_t, est_deg):
    t, _, err = align_and_error(gt_t, gt_deg, est_t, est_deg)
    abs_err = np.abs(err[:, :2])
    # yaw 没有磁力计观测量，只统计末段平均漂移，不作为精度指标
    tail = slice(int(0.8 * len(err)), len(err))
    return {
        'samples': int(len(t)),
        'rmse_roll': float(math.sqrt(float(np.mean(err[:, 0] ** 2)))),
        'rmse_pitch': float(math.sqrt(float(np.mean(err[:, 1] ** 2)))),
        'max': float(np.max(abs_err)),
        'p95': float(np.percentile(abs_err, 95)),
        'yaw_drift': float(np.mean(np.abs(err[tail, 2]))),
    }


def evaluate_all(results_dir):
    """返回 {profile: {method: metrics}}，以及原始序列。"""
    metrics = {}
    series = {}
    for profile in PROFILES:
        gt_path = os.path.join(results_dir, '%s_ground_truth.csv' % profile)
        if not os.path.exists(gt_path):
            continue
        gt_t, gt_deg = load_series(gt_path)
        metrics[profile] = {}
        series[profile] = {'truth': (gt_t, gt_deg)}
        for method in METHODS:
            est_path = os.path.join(results_dir, '%s_%s.csv' % (profile, method))
            if not os.path.exists(est_path):
                continue
            est_t, est_deg = load_series(est_path)
            series[profile][method] = (est_t, est_deg)
            metrics[profile][method] = compute_metrics(gt_t, gt_deg, est_t, est_deg)
    return metrics, series


def save_metrics(results_dir, metrics):
    path = os.path.join(results_dir, 'metrics.csv')
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['profile', 'method', 'samples', 'rmse_roll_deg',
                         'rmse_pitch_deg', 'max_deg', 'p95_deg', 'yaw_drift_deg'])
        for profile in PROFILES:
            for method in METHODS:
                m = metrics.get(profile, {}).get(method)
                if not m:
                    continue
                writer.writerow([profile, method, m['samples'],
                                 '%.3f' % m['rmse_roll'], '%.3f' % m['rmse_pitch'],
                                 '%.3f' % m['max'], '%.3f' % m['p95'], '%.2f' % m['yaw_drift']])
    return path


def print_table(metrics):
    print('| 场景 | 方法 | 样本数 | roll RMSE (°) | pitch RMSE (°) | 最大误差 (°) | p95 (°) | yaw 末段漂移 (°) |')
    print('|------|------|--------|---------------|----------------|--------------|---------|------------------|')
    for profile in PROFILES:
        for method in METHODS:
            m = metrics.get(profile, {}).get(method)
            if not m:
                continue
            print('| %s | %s | %d | %.2f | %.2f | %.2f | %.2f | %.1f |' % (
                profile, method, m['samples'], m['rmse_roll'],
                m['rmse_pitch'], m['max'], m['p95'], m['yaw_drift']))


def bench(iterations=50000):
    """单步计算耗时（纯算法部分，不含 ROS 通信）。"""
    q0 = quat_from_euler(0.1, -0.05, 0.2)
    gyro = [0.01, 0.02, -0.01]
    acc = gravity_in_body(q0)
    results = {}
    for method in METHODS:
        q = q0.copy()
        start = time.perf_counter()
        for _ in range(iterations):
            if method == 'complementary':
                q = complementary_step(q, gyro, acc, 0.005, 0.98)
            else:
                q = madgwick_step(q, gyro, acc, 0.005, 0.05)
        elapsed = time.perf_counter() - start
        results[method] = elapsed / iterations * 1e6
    print('单步耗时（%d 次平均）：' % iterations)
    for method in METHODS:
        print('  %-14s %.1f us' % (method, results[method]))
    return results


def plot(results_dir, figure_path, curve_profile):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    metrics, series = evaluate_all(results_dir)
    if curve_profile not in series:
        raise SystemExit('缺少场景 %s 的数据' % curve_profile)

    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5))
    for ax, axis_name, axis_index in ((axes[0][0], 'roll', 0), (axes[0][1], 'pitch', 1)):
        gt_t, gt_deg = series[curve_profile]['truth']
        ax.plot(gt_t, gt_deg[:, axis_index], color=COLORS['truth'], linewidth=2.0,
                label='ground truth')
        for method in METHODS:
            if method not in series[curve_profile]:
                continue
            est_t, est_deg = series[curve_profile][method]
            ax.plot(est_t, est_deg[:, axis_index], color=COLORS[method], linewidth=1.2,
                    label=method)
        ax.set_title('%s (%s profile)' % (axis_name.capitalize(), curve_profile))
        ax.set_xlabel('time (s)')
        ax.set_ylabel('angle (deg)')
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=9)

    ax = axes[1][0]
    gt_t, gt_deg = series[curve_profile]['truth']
    for method in METHODS:
        if method not in series[curve_profile]:
            continue
        est_t, est_deg = series[curve_profile][method]
        t, _, err = align_and_error(gt_t, gt_deg, est_t, est_deg)
        ax.plot(t, np.abs(err[:, 0]), color=COLORS[method], linewidth=1.0,
                label='%s |roll err|' % method)
    ax.set_title('Absolute roll error (%s profile)' % curve_profile)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('error (deg)')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=9)

    ax = axes[1][1]
    profiles = [p for p in PROFILES if p in metrics]
    width = 0.35
    x = np.arange(len(profiles))
    for index, method in enumerate(METHODS):
        values = []
        for profile in profiles:
            m = metrics.get(profile, {}).get(method)
            values.append(0.5 * (m['rmse_roll'] + m['rmse_pitch']) if m else 0.0)
        ax.bar(x + (index - 0.5) * width, values, width, color=COLORS[method], label=method)
    ax.set_xticks(x)
    ax.set_xticklabels(profiles)
    ax.set_title('Mean roll/pitch RMSE by profile')
    ax.set_ylabel('RMSE (deg)')
    ax.grid(axis='y', alpha=0.3)
    ax.legend(loc='upper left', fontsize=9)

    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(figure_path)), exist_ok=True)
    fig.savefig(figure_path, dpi=120)
    print('对比图已保存到 %s' % figure_path)


def main():
    parser = argparse.ArgumentParser(description='IMU 姿态解算误差评估')
    parser.add_argument('--results', default='results', help='CSV 所在目录')
    parser.add_argument('--figure', default='', help='输出对比图的路径（可选）')
    parser.add_argument('--profile', default='fast', help='绘制曲线的场景')
    parser.add_argument('--bench', action='store_true', help='只测单步耗时')
    args = parser.parse_args()

    if args.bench:
        bench()
        return

    metrics, _ = evaluate_all(args.results)
    if not metrics:
        raise SystemExit('在 %s 下没有找到结果 CSV' % args.results)
    print_table(metrics)
    print('指标已保存到 %s' % save_metrics(args.results, metrics))
    if args.figure:
        plot(args.results, args.figure, args.profile)


if __name__ == '__main__':
    main()
