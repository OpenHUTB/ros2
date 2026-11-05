"""Learn temporal cross-channel consistency rather than a particular flight path."""
import json
from pathlib import Path
import numpy as np
import torch
from torch import nn
from train_evaluate import KEYS


class ConsistencyNet(nn.Module):
    def __init__(self, variant='gru'):
        super().__init__(); self.variant = variant
        if variant == 'gru':
            self.encoder = nn.GRU(6, 16, batch_first=True)
            self.head = nn.Linear(16, 1)
        else:
            self.head = nn.Sequential(nn.Linear(6, 16), nn.Tanh(), nn.Linear(16, 1))

    def forward(self, x):
        if self.variant == 'gru':
            _, h = self.encoder(x); return self.head(h[-1]).squeeze(-1)
        return self.head(x[:, -1]).squeeze(-1)


def residual(previous, current, dt):
    average = (previous + current) / 2
    return np.r_[(current[:3] - previous[:3]) / dt - average[3:6],
                 (current[3:6] - previous[3:6]) / dt - average[6:9]]


def extract(rows, history=8):
    past, windows, indices, rules = [], [], [], []
    previous, source = None, None
    for i, row in enumerate(rows):
        stamp = int(row['source_ns']) if int(row['delivered']) else None
        fresh = stamp is not None and (source is None or stamp > source)
        rules.append(not fresh)
        if not fresh:
            previous, past = None, []; continue
        z = np.array([float(row[k]) for k in KEYS], dtype=np.float64)
        if previous is not None:
            dt = (stamp - source) / 1e9
            if .01 <= dt <= .15:
                past.append(residual(previous, z, dt))
                if len(past) >= history:
                    windows.append(np.array(past[-history:])); indices.append(i)
            else: past = []
        previous, source = z, stamp
    return np.asarray(windows, dtype=np.float32).reshape(-1, history, 6), np.asarray(indices, int), np.array(rules)


def transform(x, scale):
    return (np.sign(x) * np.log1p(np.abs(x) / scale)).astype(np.float32)


class ConsistencyDetector:
    def __init__(self, folder):
        self.config = json.loads((Path(folder) / 'config.json').read_text())
        self.model = ConsistencyNet(self.config['variant'])
        self.model.load_state_dict(torch.load(Path(folder) / 'model.pt', weights_only=True))
        self.model.eval(); torch.set_num_threads(2)
        self.previous = None; self.source = None; self.history = []; self.run = 0; self.arrival = 0

    def step(self, row):
        c = self.config; score = None; valid = False; reason = 'warming_up'
        try:
            arrival = int(row['arrival_ns']); delivered = int(row['delivered'])
            if arrival < 0 or delivered not in (0, 1): raise ValueError()
            stamp = int(row['source_ns']) if delivered else None
            z = np.array([float(row[k]) for k in KEYS]) if delivered else None
            if delivered and (stamp < 0 or not np.isfinite(z).all()): raise ValueError()
            self.arrival = arrival
            valid = delivered and (self.source is None or stamp > self.source)
            if not valid:
                reason = 'missing_packet' if not delivered else 'stale_timestamp'
                self.previous, self.history = None, []; abnormal = True
            else:
                abnormal = False
                if self.previous is not None:
                    dt = (stamp - self.source) / 1e9
                    if .01 <= dt <= .15:
                        self.history.append(residual(self.previous, z, dt))
                        self.history = self.history[-c['history']:]
                        if len(self.history) == c['history']:
                            x = transform(np.array(self.history), np.array(c['scale']))
                            with torch.no_grad(): score = float(self.model(torch.from_numpy(x[None])).item())
                            abnormal = score > c['threshold']
                            reason = 'model_residual' if abnormal else 'normal'
                    else:
                        self.history = []; reason = 'time_gap'
                self.previous, self.source = z, stamp
        except (KeyError, TypeError, ValueError, OverflowError):
            reason = 'invalid_payload'; abnormal = True; self.previous, self.history = None, []
        self.run = self.run + 1 if abnormal else 0
        return {'arrival_ns': self.arrival, 'score': score, 'threshold': c['threshold'],
                'alarm': bool(self.run >= c['persistence'] or reason == 'invalid_payload'),
                'reason': reason, 'data_valid': bool(valid), 'model_ready': score is not None,
                'consecutive': self.run}
