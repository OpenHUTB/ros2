"""Evaluate the imputer against baselines and write metrics + figures."""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from . import data, numpy_infer

try:                       # torch is optional for evaluation (numpy fallback)
    import torch
    from .model import Imputer
    HAS_TORCH = True
except ImportError:        # pragma: no cover - exercised by the numpy-only path
    torch = None
    Imputer = None
    HAS_TORCH = False


def load_model(folder):
    """Load the model; returns (backend, payload, cfg).

    With torch available the checkpoint is used directly.  Without it we fall
    back to ``weights.npz`` and a hand-written GRU forward pass, so reviewers
    without a 200 MB torch wheel can still check the numbers.
    """
    folder = Path(folder)
    cfg = json.loads((folder / 'config.json').read_text())
    if HAS_TORCH:
        net = Imputer(cfg['hidden'])
        net.load_state_dict(torch.load(folder / 'model.pt', weights_only=True))
        net.eval()
        torch.set_num_threads(2)
        return 'torch', net, cfg
    weights, cfg = numpy_infer.load(folder)
    return 'numpy', weights, cfg


def predict(backend, payload, cfg, xs, ys, ms):
    if backend == 'numpy':
        return numpy_infer.predict(payload, cfg, xs, ys, ms)
    base = data.baselines(ys, ms, 'zoh')
    mu, sd = np.asarray(cfg['mean'], np.float32), np.asarray(cfg['std'], np.float32)
    x = data.encode(xs, ms, mu, sd)
    with torch.no_grad():
        res = payload(torch.tensor(x)).numpy() * sd
    return base + res


def rmse(pred, truth, mask):
    err = (pred - truth)[~mask]
    return float(np.sqrt((err[:, :3] ** 2).mean())), float(np.sqrt((err[:, 3:] ** 2).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default='flights')
    ap.add_argument('--model', default='models/imputer_42')
    ap.add_argument('--output', default='results')
    a = ap.parse_args()

    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    backend, payload, cfg = load_model(a.model)
    xs, ys, ms, gs = data.dataset(Path(a.data), 'test', np.random.default_rng(909))

    preds = {
        'imputer': predict(backend, payload, cfg, xs, ys, ms),
        'zoh': data.baselines(ys, ms, 'zoh'),
        'linear': data.baselines(ys, ms, 'linear'),
        'const_vel': data.baselines(ys, ms, 'const_vel'),
    }
    summary = {}
    for name, p in preds.items():
        pos, vel = rmse(p, ys, ms)
        summary[name] = {'position_rmse_m': pos, 'velocity_rmse_mps': vel}

    buckets = [(1, 4), (5, 8), (9, 12), (13, 16), (17, 20)]
    per_gap = []
    for lo, hi in buckets:
        sel = (gs >= lo) & (gs <= hi)
        if not sel.any():
            continue
        row = {'gap_lo': lo, 'gap_hi': hi, 'windows': int(sel.sum())}
        for name, p in preds.items():
            pos, vel = rmse(p[sel], ys[sel], ms[sel])
            row[name] = {'position_rmse_m': pos, 'velocity_rmse_mps': vel}
        per_gap.append(row)

    payload_out = {'windows': int(len(xs)), 'backend': backend, 'summary': summary, 'per_gap': per_gap}
    (out / 'metrics.json').write_text(json.dumps(payload_out, indent=2))

    # figure 1: RMSE vs gap length
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    mids = [(r['gap_lo'] + r['gap_hi']) / 2 for r in per_gap]
    for name, label, color in [('imputer', 'GRU imputer', '#4a90d9'),
                               ('zoh', 'zero-order hold', '#e08a3c'),
                               ('linear', 'linear interpolation', '#3a9e6f'),
                               ('const_vel', 'constant velocity', '#8a6fbf')]:
        ax.plot(mids, [r[name]['position_rmse_m'] for r in per_gap], 'o-', label=label, color=color)
    ax.set(xlabel='Gap length (samples, 20 Hz)', ylabel='Position RMSE (m)',
           title='Reconstruction error vs gap length')
    ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(out / 'gap_length.png', dpi=160); plt.close(fig)

    # figure 2: one example trajectory with reconstruction
    i = int(np.argmax(gs))
    t = np.arange(data.WINDOW) * data.DT
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8))
    for ax, dim, label in zip(axes, [0, 2], ['x (m)', 'z (m)']):
        ax.plot(t, ys[i, :, dim], 'k-', lw=2, label='truth')
        ax.plot(t[ms[i]], ys[i, ms[i], dim], 'ko', ms=4, label='observed')
        ax.plot(t, preds['imputer'][i, :, dim], '--', color='#4a90d9', label='imputer')
        ax.plot(t, preds['zoh'][i, :, dim], ':', color='#e08a3c', label='ZOH')
        ax.set(xlabel='Time in window (s)', ylabel=label)
        ax.grid(alpha=.25)
    axes[0].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(out / 'example_window.png', dpi=160); plt.close(fig)

    print(json.dumps(payload_out['summary'], indent=2))
    print('windows', payload_out['windows'], '| backend:', backend)


if __name__ == '__main__':
    main()
