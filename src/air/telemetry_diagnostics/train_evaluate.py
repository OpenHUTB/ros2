"""Causal one-step prediction benchmark; never loads evaluation truth as features."""
import argparse
import csv
import json
import random
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

KEYS = ['px', 'py', 'pz', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']


class Predictor(nn.Module):
    def __init__(self, variant='gru'):
        super().__init__()
        self.variant = variant
        if variant == 'gru':
            self.encoder = nn.GRU(9, 32, batch_first=True)
            self.head = nn.Linear(32, 6)
        else:
            self.head = nn.Sequential(nn.Linear(9, 32), nn.ReLU(), nn.Linear(32, 6))

    def forward(self, x):
        if self.variant == 'gru':
            _, h = self.encoder(x)
            return self.head(h[-1])
        return self.head(x[:, -1])


def load_episode(folder, history=12):
    with (folder / 'observations.csv').open() as f:
        rows = list(csv.DictReader(f))
    n = len(rows)
    t = np.array([int(r['arrival_ns']) for r in rows], dtype=np.int64)
    features, xs, ys, indices, rules, physics = [], [], [], [], [], []
    previous = None
    last_source = None
    for i, r in enumerate(rows):
        delivered = int(r['delivered']) == 1
        # Replay scheduler supplies no-packet ticks; online ROS uses a timer instead.
        fresh = delivered and (last_source is None or int(r['source_ns']) > last_source)
        rules.append(not fresh)
        if not fresh:
            features, previous = [], None
            continue
        source = int(r['source_ns'])
        z = np.array([float(r[k]) for k in KEYS], dtype=np.float32)
        if previous is not None:
            dt = (source - last_source) / 1e9
            if not .01 <= dt <= .15:
                features = []
            else:
                delta = z[:6] - previous[:6]
                if len(features) >= history:
                    xs.append(np.array(features[-history:]))
                    ys.append(delta)
                    indices.append(i)
                    physics.append(delta - np.r_[previous[3:6] * dt, previous[6:9] * dt])
                features.append(np.r_[(z[:3] - previous[:3]) / dt, z[3:9]])
        previous, last_source = z, source
    return {'x': np.asarray(xs, dtype=np.float32).reshape(-1, history, 9),
            'y': np.asarray(ys, dtype=np.float32).reshape(-1, 6),
            'physics': np.asarray(physics, dtype=np.float32).reshape(-1, 6),
            'indices': np.array(indices, dtype=int), 'rules': np.array(rules),
            'time': (t - t[0]) / 1e9, 'n': n}


def sustained(flags, count):
    run = 0
    result = []
    for flag in flags:
        run = run + 1 if flag else 0
        result.append(run >= count)
    return np.array(result)


def evaluate(records, episodes, root, scores, threshold, persistence):
    details = []
    tp = fp = fn = 0
    healthy_seconds = 0.
    healthy_alarm_events = 0
    delays = []
    for record, ep, score in zip(records, episodes, scores):
        if record['split'] != 'test':
            continue
        with (root / record['path'] / 'evaluation_only.csv').open() as f:
            labels = np.array([int(r['anomaly']) for r in csv.DictReader(f)], dtype=bool)
        alarm = sustained((score > threshold) | ep['rules'], persistence)
        starts = np.flatnonzero(alarm & ~np.r_[False, alarm[:-1]])
        # One injected event per scenario; a persistent earlier false alarm is not a detection.
        inside = [i for i in starts if labels[i]]
        outside = [i for i in starts if not labels[i]]
        detected = bool(inside)
        tp += int(labels.any() and detected)
        fn += int(labels.any() and not detected)
        fp += len(outside)
        dt = np.diff(ep['time'], append=ep['time'][-1] + np.median(np.diff(ep['time'])))
        healthy_seconds += float(dt[~labels].sum())
        healthy_alarm_events += len(outside)
        delay = float(ep['time'][inside[0]] - ep['time'][np.flatnonzero(labels)[0]]) if inside else None
        if delay is not None:
            delays.append(delay)
        details.append({'episode': record['episode'], 'kind': record['kind'],
                        'detected': detected, 'false_alarm_events': len(outside), 'delay_s': delay})
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    return {'event_precision': precision, 'event_recall': recall,
            'event_f1': 2 * precision * recall / max(precision + recall, 1e-12),
            'missed_events': fn, 'false_alarms_per_min': healthy_alarm_events / (healthy_seconds / 60),
            'mean_detected_delay_s': float(np.mean(delays)) if delays else None, 'details': details}


def run(root, output, seed, variant, epochs):
    if (output / 'model.pt').exists():
        raise FileExistsError('Use a new --output directory; existing experiment models are preserved')
    torch.set_num_threads(3)
    torch.manual_seed(seed); np.random.seed(seed); random.seed(seed)
    records = json.loads((root / 'manifest.json').read_text())['records']
    episodes = [load_episode(root / r['path']) for r in records]
    def gather(split, key):
        return np.concatenate([e[key] for r, e in zip(records, episodes)
                               if r['split'] == split and r['kind'] == 'clean'])
    tx, ty = gather('train', 'x'), gather('train', 'y')
    vx, vy = gather('validation', 'x'), gather('validation', 'y')
    xm, xs = tx.mean((0, 1)), np.maximum(tx.std((0, 1)), .01)
    ym, ys = ty.mean(0), np.maximum(ty.std(0), .001)
    transform = lambda x: torch.from_numpy((x - xm) / xs)
    loader = DataLoader(TensorDataset(transform(tx), torch.from_numpy((ty - ym) / ys)),
                        batch_size=128, shuffle=True)
    model = Predictor(variant)
    optimizer = torch.optim.Adam(model.parameters(), lr=.001)
    output.mkdir(parents=True, exist_ok=True)
    best, history = float('inf'), []
    for epoch in range(epochs):
        model.train(); total = 0
        for x, y in loader:
            optimizer.zero_grad(); loss = ((model(x) - y) ** 2).mean()
            loss.backward(); nn.utils.clip_grad_norm_(model.parameters(), 1.)
            optimizer.step(); total += loss.item() * len(x)
        model.eval()
        with torch.no_grad():
            vl = ((model(transform(vx)) - torch.from_numpy((vy - ym) / ys)) ** 2).mean().item()
        history.append({'epoch': epoch + 1, 'train_loss': total / len(tx), 'validation_loss': vl})
        if vl < best:
            best = vl
            torch.save(model.state_dict(), output / 'model.pt')
    model.load_state_dict(torch.load(output / 'model.pt', weights_only=True))
    def residual(ep):
        with torch.no_grad():
            return ep['y'] - (model(transform(ep['x'])).numpy() * ys + ym)
    validation_residual = np.concatenate([residual(e) for r, e in zip(records, episodes)
                                         if r['split'] == 'validation' and r['kind'] == 'clean'])
    scale = np.maximum(np.sqrt((validation_residual ** 2).mean(0)), .001)
    scores = []
    for ep in episodes:
        s = np.full(ep['n'], -np.inf)
        s[ep['indices']] = np.sqrt(((residual(ep) / scale) ** 2).mean(1))
        scores.append(s)
    val_scores = np.concatenate([s[np.isfinite(s)] for r, s in zip(records, scores)
                                 if r['split'] == 'validation' and r['kind'] == 'clean'])
    threshold = float(np.quantile(val_scores, .995))
    config = {'variant': variant, 'seed': seed, 'history': 12, 'xm': xm.tolist(),
              'xs': xs.tolist(), 'ym': ym.tolist(), 'ys': ys.tolist(),
              'residual_scale': scale.tolist(), 'threshold': threshold, 'persistence': 3}
    (output / 'config.json').write_text(json.dumps(config, indent=2))
    (output / 'loss.json').write_text(json.dumps(history, indent=2))
    result = evaluate(records, episodes, root, scores, threshold, 3)
    result['without_persistence'] = evaluate(records, episodes, root, scores, threshold, 1)
    # Physical residual baseline calibrated on the same clean validation episodes.
    pval = np.concatenate([e['physics'] for r, e in zip(records, episodes)
                           if r['split'] == 'validation' and r['kind'] == 'clean'])
    pscale = np.maximum(np.sqrt((pval ** 2).mean(0)), .001)
    pscores = []
    for e in episodes:
        s = np.full(e['n'], -np.inf)
        s[e['indices']] = np.sqrt(((e['physics'] / pscale) ** 2).mean(1))
        pscores.append(s)
    pv = np.concatenate([s[np.isfinite(s)] for r, s in zip(records, pscores)
                         if r['split'] == 'validation' and r['kind'] == 'clean'])
    result['physics_baseline'] = evaluate(records, episodes, root, pscores, float(np.quantile(pv, .995)), 3)
    (output / 'metrics.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k not in ['details', 'without_persistence', 'physics_baseline']}))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--data', type=Path, default=Path('data/prepared'))
    p.add_argument('--output', type=Path, default=Path('models/retrained_gru_42'))
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--variant', choices=['gru', 'mlp'], default='gru')
    p.add_argument('--epochs', type=int, default=30)
    args = p.parse_args()
    run(args.data, args.output, args.seed, args.variant, args.epochs)
