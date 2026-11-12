"""NumPy-only inference for the GRU imputer.

The shipped weights are tiny (8,790 parameters), so a reviewer without PyTorch
can still check the reported numbers.  ``export_weights`` converts a trained
``model.pt`` into ``weights.npz``; ``predict`` reproduces exactly what
``evaluate.py`` does with torch.
"""
import json
from pathlib import Path

import numpy as np

from . import data


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def export_weights(net, folder):
    """Save a torch ``Imputer`` as a numpy-friendly npz file."""
    folder = Path(folder)
    sd = {k: v.detach().cpu().numpy() for k, v in net.state_dict().items()}
    np.savez_compressed(
        folder / 'weights.npz',
        weight_ih_l0=sd['gru.weight_ih_l0'], weight_hh_l0=sd['gru.weight_hh_l0'],
        bias_ih_l0=sd['gru.bias_ih_l0'], bias_hh_l0=sd['gru.bias_hh_l0'],
        head_weight=sd['head.weight'], head_bias=sd['head.bias'],
    )


def load(folder):
    """Load the npz weights and config of a model folder."""
    folder = Path(folder)
    cfg = json.loads((folder / 'config.json').read_text())
    with np.load(folder / 'weights.npz') as npz:
        weights = {k: npz[k].astype(np.float64) for k in npz.files}
    return weights, cfg


def forward(weights, x):
    """GRU + linear head, following PyTorch's gate order (r, z, n)."""
    w_ih = weights['weight_ih_l0']
    w_hh = weights['weight_hh_l0']
    b_ih = weights['bias_ih_l0']
    b_hh = weights['bias_hh_l0']
    hidden = w_hh.shape[1]
    batch, steps, _ = x.shape
    h = np.zeros((batch, hidden), dtype=np.float64)
    outs = np.empty((batch, steps, weights['head_weight'].shape[0]), dtype=np.float64)
    for t in range(steps):
        gi = x[:, t] @ w_ih.T + b_ih
        gh = h @ w_hh.T + b_hh
        r = sigmoid(gi[:, :hidden] + gh[:, :hidden])
        z = sigmoid(gi[:, hidden:2 * hidden] + gh[:, hidden:2 * hidden])
        n = np.tanh(gi[:, 2 * hidden:] + r * gh[:, 2 * hidden:])
        h = (1.0 - z) * n + z * h
        outs[:, t] = h @ weights['head_weight'].T + weights['head_bias']
    return outs


def predict(weights, cfg, xs, ys, ms):
    """Reconstruct a batch of windows the same way ``evaluate.predict`` does."""
    base = data.baselines(ys, ms, 'zoh')
    mu = np.asarray(cfg['mean'], np.float32)
    sd = np.asarray(cfg['std'], np.float32)
    x = data.encode(xs, ms, mu, sd)
    residual = forward(weights, x.astype(np.float64)) * sd.astype(np.float64)
    return base + residual
