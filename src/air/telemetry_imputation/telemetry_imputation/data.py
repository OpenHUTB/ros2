"""Causal windows with masks for telemetry imputation."""
from pathlib import Path
import json
import numpy as np

WINDOW = 40          # 2 s at 20 Hz
CHANNELS = ['px', 'py', 'pz', 'vx', 'vy', 'vz']
OBSERVED = CHANNELS + ['thrust']
DT = 0.05


def load_flight(path):
    raw = np.loadtxt(path, delimiter=',', skiprows=1)
    idx = {name: i for i, name in enumerate(
        ['t', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'thrust'])}
    t = raw[:, idx['t']]
    state = np.stack([raw[:, idx[c]] for c in CHANNELS], axis=1).astype(np.float32)
    thrust = raw[:, idx['thrust']].astype(np.float32)
    return t, state, thrust


def make_windows(state, thrust, t, rng, gap=(1, 20), mode='contiguous'):
    """Slice a flight into windows and punch a hole; returns windows + masks."""
    n = state.shape[0]
    xs, ys, masks, gaps = [], [], [], []
    if n < WINDOW + 1:
        return None
    for start in range(0, n - WINDOW, WINDOW // 4):
        sl = slice(start, start + WINDOW)
        s = state[sl]
        f = thrust[sl]
        dt = np.diff(t[sl], prepend=t[start] - DT)
        mask = np.ones(WINDOW, bool)
        if mode == 'contiguous':
            length = int(rng.integers(gap[0], gap[1] + 1))
            begin = int(rng.integers(0, WINDOW - length + 1))
            mask[begin:begin + length] = False
            g = length
        else:
            rate = float(rng.uniform(*gap)) / WINDOW
            mask = rng.random(WINDOW) > rate
            g = int((~mask).sum())
        if (~mask).sum() == 0:
            continue
        x = np.concatenate([s * mask[:, None], f[:, None], mask[:, None].astype(np.float32),
                            dt[:, None].astype(np.float32)], axis=1)
        xs.append(x); ys.append(s); masks.append(mask); gaps.append(g)
    if not xs:
        return None
    return (np.stack(xs).astype(np.float32), np.stack(ys).astype(np.float32),
            np.stack(masks), np.asarray(gaps))


def dataset(root, split, rng, gap=(1, 20), mode='contiguous'):
    root = Path(root)
    manifest = json.loads((root / 'manifest.json').read_text())
    parts = []
    for item in manifest:
        if item['split'] != split:
            continue
        t, state, thrust = load_flight(root / item['file'])
        out = make_windows(state, thrust, t, rng, gap, mode)
        if out:
            parts.append(out)
    xs = np.concatenate([p[0] for p in parts])
    ys = np.concatenate([p[1] for p in parts])
    ms = np.concatenate([p[2] for p in parts])
    gs = np.concatenate([p[3] for p in parts])
    return xs, ys, ms, gs


def normalizer(xs, ys, ms):
    """Scale from observed entries only."""
    obs = ys[ms]
    return obs.mean(0).astype(np.float32), obs.std(0).clip(1e-3).astype(np.float32)


def encode(xs, mask, mu, sd):
    """Standardise observed channels, then force missing entries to zero.

    Masking after scaling keeps "missing" at exactly 0 in input space; scaling
    first would turn a missing frame into -mu/sd, which is a non-zero constant.
    """
    x = xs.astype(np.float32).copy()
    x[:, :, :6] = ((x[:, :, :6] - mu) / sd) * mask[:, :, None]
    return x


def baselines(y, mask, kind):
    """Reference reconstructions of a batch of windows (y: B×W×6, mask: B×W)."""
    b, w, _ = y.shape
    out = np.zeros_like(y)
    for i in range(b):
        known = np.where(mask[i])[0]
        if kind == 'zoh':
            out[i] = y[i, known[0]] if len(known) else 0
            for k in range(w):
                prev = known[known <= k]
                if len(prev):
                    out[i, k] = y[i, prev[-1]]
        elif kind == 'linear':
            for k in range(w):
                prev = known[known < k]
                nxt = known[known > k]
                if len(prev):
                    p0 = prev[-1]
                    if len(nxt):
                        p1 = nxt[0]
                        wgt = (k - p0) / (p1 - p0)
                        out[i, k] = y[i, p0] * (1 - wgt) + y[i, p1] * wgt
                    else:
                        if len(prev) > 1:
                            step = max(p0 - prev[-2], 1)
                            slope = (y[i, p0] - y[i, prev[-2]]) / step
                        else:
                            slope = 0
                        out[i, k] = y[i, p0] + slope * (k - p0)
        elif kind == 'const_vel':
            for k in range(w):
                prev = known[known <= k]
                if len(prev):
                    p0 = prev[-1]
                    out[i, k] = y[i, p0]
                    if len(prev) >= 2:
                        out[i, k, :3] = y[i, p0, :3] + y[i, p0, 3:] * DT * (k - p0)
        else:
            raise ValueError(kind)
    return out
