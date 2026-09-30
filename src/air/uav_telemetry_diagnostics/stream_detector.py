"""Stateful causal detector shared by ROS inference and equivalence tests."""
import json
from pathlib import Path
import numpy as np
import torch
from train_evaluate import KEYS, Predictor


def load_detector(folder):
    config = json.loads((Path(folder) / 'config.json').read_text())
    if config.get('method') == 'consistency':
        from consistency import ConsistencyDetector
        return ConsistencyDetector(folder)
    return Detector(folder)


class Detector:
    def __init__(self, folder):
        folder = Path(folder)
        self.config = json.loads((folder / 'config.json').read_text())
        self.model = Predictor(self.config['variant'])
        self.model.load_state_dict(torch.load(folder / 'model.pt', weights_only=True))
        self.model.eval()
        torch.set_num_threads(2)
        self.features = []
        self.previous = None
        self.last_source = None
        self.run = 0

    def step(self, row):
        try:
            if not isinstance(row, dict) or int(row['delivered']) not in (0, 1):
                raise ValueError('Invalid envelope')
            arrival = int(row['arrival_ns'])
            if arrival < 0:
                raise ValueError('Invalid arrival time')
            if int(row['delivered']):
                if int(row['source_ns']) < 0 or not all(np.isfinite(float(row[k])) for k in KEYS):
                    raise ValueError('Invalid payload')
        except (KeyError, ValueError, TypeError, OverflowError):
            self.features, self.previous = [], None
            self.run += 1
            return {'arrival_ns': 0, 'score': None, 'threshold': self.config['threshold'],
                    'alarm': True, 'reason': 'invalid_payload', 'consecutive': self.run,
                    'data_valid': False, 'model_ready': False}
        c = self.config
        source = int(row['source_ns']) if int(row['delivered']) else None
        fresh = source is not None and (self.last_source is None or source > self.last_source)
        score = None
        reason = 'warming_up'
        if not fresh:
            reason = 'missing_packet' if source is None else 'stale_timestamp'
            self.features, self.previous = [], None
            abnormal = True
        else:
            z = np.array([float(row[k]) for k in KEYS], dtype=np.float32)
            abnormal = False
            if self.previous is not None:
                dt = (source - self.last_source) / 1e9
                if not .01 <= dt <= .15:
                    self.features = []
                    reason = 'time_gap'
                else:
                    if len(self.features) >= c['history']:
                        x = np.array(self.features[-c['history']:], dtype=np.float32)
                        x = ((x - np.array(c['xm'], dtype=np.float32)) /
                             np.array(c['xs'], dtype=np.float32))
                        with torch.no_grad():
                            y = self.model(torch.from_numpy(x[None])).numpy()[0]
                        predicted = y * np.array(c['ys']) + np.array(c['ym'])
                        residual = z[:6] - self.previous[:6] - predicted
                        score = float(np.sqrt(np.mean((residual / c['residual_scale']) ** 2)))
                        abnormal = score > c['threshold']
                        reason = 'model_residual' if abnormal else 'normal'
                    self.features.append(np.r_[(z[:3] - self.previous[:3]) / dt, z[3:9]])
                    self.features = self.features[-c['history']:]
            self.previous, self.last_source = z, source
        self.run = self.run + 1 if abnormal else 0
        return {'arrival_ns': int(row['arrival_ns']), 'score': score,
                'threshold': c['threshold'], 'alarm': self.run >= c['persistence'],
                'reason': reason, 'consecutive': self.run, 'data_valid': fresh,
                'model_ready': score is not None}
