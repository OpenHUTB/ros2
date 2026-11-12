"""把训练与演示的指标画成文档用的图。

图上所有文字都是英文：matplotlib 默认字体没有中文字形，混进中文会变成方块。
"""
import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_json(path):
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def plot_offset_series(payload, path):
    """视觉偏移与地图真值随时间的变化，看两条线贴不贴。"""
    series = payload.get('series', {})
    visual = np.asarray(series.get('visual_offset', []), dtype=float)
    truth = np.asarray(series.get('map_offset', []), dtype=float)
    if visual.size == 0:
        return False

    plt.figure(figsize=(7.2, 3.0))
    plt.plot(truth, label='map ground truth', linewidth=1.4)
    plt.plot(visual, label='camera estimate', linewidth=1.4, alpha=0.85)
    plt.xlabel('frame')
    plt.ylabel('lateral offset (m)')
    plt.title('Lateral offset: camera vs map')
    plt.grid(alpha=0.3)
    plt.legend(loc='best', fontsize=8)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    return True


def plot_scatter(payload, path):
    """预测值对真值的散点，越靠近对角线越准。"""
    series = payload.get('series', {})
    visual = np.asarray(series.get('visual_offset', []), dtype=float)
    truth = np.asarray(series.get('map_offset', []), dtype=float)
    if visual.size == 0:
        return False

    limit = max(1.0, float(np.abs(truth).max()) * 1.15)
    plt.figure(figsize=(3.6, 3.6))
    plt.scatter(truth, visual, s=8, alpha=0.6)
    plt.plot([-limit, limit], [-limit, limit], 'r--', linewidth=1.0,
             label='ideal')
    plt.xlim(-limit, limit)
    plt.ylim(-limit, limit)
    plt.xlabel('map offset (m)')
    plt.ylabel('camera offset (m)')
    plt.title('Estimate vs ground truth')
    plt.grid(alpha=0.3)
    plt.legend(loc='upper left', fontsize=8)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    return True


def plot_speed_buckets(payload, path):
    """按车速分档的平均绝对误差。"""
    buckets = payload.get('metrics', {}).get('by_speed', {})
    if not buckets:
        return False

    labels = list(buckets.keys())
    values = [buckets[name]['mae_offset'] for name in labels]
    plt.figure(figsize=(4.8, 2.8))
    bars = plt.bar(range(len(labels)), values, color='#4c72b0')
    plt.xticks(range(len(labels)), labels, fontsize=8)
    plt.ylabel('offset MAE (m)')
    plt.title('Error by speed')
    for bar, value in zip(bars, values):
        plt.text(bar.get_x() + bar.get_width() / 2, value,
                 '%.3f' % value, ha='center', va='bottom', fontsize=8)
    plt.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    return True


def main():
    parser = argparse.ArgumentParser(description='生成文档配图')
    parser.add_argument('--metrics', default=os.path.join('out', 'demo_metrics.json'),
                        help='演示指标 json')
    parser.add_argument('--out-dir', default=os.path.join('figures'),
                        help='图片输出目录')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    if not os.path.isfile(args.metrics):
        print('找不到 %s，先跑 main.py --demo --save-metrics' % args.metrics)
        return 1

    payload = load_json(args.metrics)
    written = []
    for name, func in (('visual_ldw_offset_series.png', plot_offset_series),
                       ('visual_ldw_scatter.png', plot_scatter),
                       ('visual_ldw_speed_error.png', plot_speed_buckets)):
        path = os.path.join(args.out_dir, name)
        if func(payload, path):
            written.append(path)
            print('写出 %s' % path)

    summary = {
        'offset_mae': payload.get('metrics', {}).get('mae_offset'),
        'offset_p90': payload.get('metrics', {}).get('p90_offset'),
        'events': payload.get('events', {}),
    }
    with open(os.path.join(args.out_dir, 'summary.json'), 'w',
              encoding='utf-8') as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    return 0 if written else 1


if __name__ == '__main__':
    raise SystemExit(main())