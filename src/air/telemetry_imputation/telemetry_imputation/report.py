"""Aggregate training and evaluation artefacts into the summary figures."""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='models/imputer_42')
    ap.add_argument('--results', default='results')
    a = ap.parse_args()

    model_dir = Path(a.model)
    out = Path(a.results)
    out.mkdir(parents=True, exist_ok=True)

    loss = np.loadtxt(model_dir / 'loss.csv', delimiter=',', skiprows=1)
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    ax.plot(loss[:, 0], loss[:, 1], label='train (masked MSE)')
    ax.plot(loss[:, 0], loss[:, 2], label='validation')
    ax.set(xlabel='Epoch', ylabel='Masked MSE (normalised residual)')
    ax.set_yscale('log'); ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(out / 'loss.png', dpi=160); plt.close(fig)

    metrics = json.loads((out / 'metrics.json').read_text())
    names = ['imputer', 'zoh', 'linear', 'const_vel']
    labels = ['GRU imputer', 'zero-order hold', 'linear interpolation', 'constant velocity']
    pos = [metrics['summary'][n]['position_rmse_m'] for n in names]
    vel = [metrics['summary'][n]['velocity_rmse_mps'] for n in names]

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.6))
    for ax, vals, title, unit in [(axes[0], pos, 'Position', 'RMSE (m)'),
                                  (axes[1], vel, 'Velocity', 'RMSE (m/s)')]:
        bars = ax.bar(range(len(names)), vals, color=['#4a90d9', '#e08a3c', '#3a9e6f', '#8a6fbf'])
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width()/2, b.get_height() * 1.02, '%.3f' % v,
                    ha='center', va='bottom', fontsize=8)
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels(labels, rotation=18, fontsize=8)
        ax.set(title=title + ' reconstruction error', ylabel=unit)
        ax.grid(axis='y', alpha=.25)
    fig.tight_layout(); fig.savefig(out / 'baselines.png', dpi=160); plt.close(fig)

    print(json.dumps({'figures': ['loss.png', 'baselines.png'], 'windows': metrics['windows']}))


if __name__ == '__main__':
    main()
