"""Frozen-model evaluation on newly collected flights, with smooth injected faults."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from telemetry_data import CHANNELS, write_csv
from train_evaluate import load_episode, sustained
from stream_detector import Detector


def scenarios(rows):
    times = np.array([int(r['timestamp_ns']) for r in rows], dtype=np.int64)
    t = (times - times[0]) / 1e9
    start, end = .35 * t[-1], .7 * t[-1]
    mask = (t >= start) & (t < end)
    phase = np.clip((t - start) / (end - start), 0, 1)
    envelope = np.sin(np.pi * phase) ** 2
    base = [{'arrival_ns': int(r['timestamp_ns']), 'source_ns': int(r['timestamp_ns']),
             'delivered': 1, **{k: float(r[k]) for k in CHANNELS}} for r in rows]
    yield 'clean', 0., base, np.zeros(len(rows), dtype=bool)
    for kind in ('smooth_position_drift', 'velocity_bias'):
        for severity in (.03, .1, .3):
            obs = [r.copy() for r in base]
            # Smooth rise and recovery, without artificial discontinuity at either boundary.
            for i, r in enumerate(obs):
                if kind == 'smooth_position_drift':
                    # Max slope of this displacement bump equals severity in m/s.
                    r['px'] += severity * (end - start) / np.pi * envelope[i]
                else:
                    r['vx'] += severity * envelope[i]
            yield kind, severity, obs, mask
    first = int(np.flatnonzero(mask)[0])
    for kind in ('freeze', 'dropout'):
        obs = [r.copy() for r in base]
        for i in np.flatnonzero(mask):
            arrival = obs[i]['arrival_ns']
            obs[i] = dict(base[first - 1], arrival_ns=arrival) if kind == 'freeze' else {
                'arrival_ns': arrival, 'source_ns': '', 'delivered': 0, **{k: '' for k in CHANNELS}}
        yield kind, 0., obs, mask


def score_events(alarm, labels, time):
    starts = np.flatnonzero(alarm & ~np.r_[False, alarm[:-1]])
    hit = [i for i in starts if labels[i]]
    delay = float(time[hit[0]] - time[np.flatnonzero(labels)[0]]) if hit else None
    return {'event_present': bool(labels.any()), 'detected': bool(hit),
            'false_alarm_events': sum(not labels[i] for i in starts),
            'delay_s': delay, 'alarm_samples': int(alarm.sum())}


def main():
    root = Path('data/prepared'); out = Path('results/holdout'); out.mkdir(parents=True, exist_ok=True)
    protocol = json.loads(Path('holdout_protocol.json').read_text())
    for name, digest in protocol['hashes'].items():
        assert hashlib.sha256((Path('models/gru_42') / name).read_bytes()).hexdigest() == digest
    # Existing validation data only: physical comparator calibrated independently of fresh tests.
    records = json.loads((root / 'manifest.json').read_text())['records']
    val = [load_episode(root / r['path']) for r in records if r['split'] == 'validation' and r['kind'] == 'clean']
    residual = np.concatenate([e['physics'] for e in val])
    scale = np.maximum(np.sqrt((residual ** 2).mean(0)), .001)
    threshold = float(np.quantile(np.sqrt(((residual / scale) ** 2).mean(1)), .995))
    details, manifest = [], []
    files = sorted(Path('data/holdout_raw').glob('*.csv'))
    assert len(files) == protocol['holdout_episodes'], 'Incomplete fresh collection'
    for raw in files:
        meta = json.loads(raw.with_suffix('.json').read_text())
        assert hashlib.sha256(raw.read_bytes()).hexdigest() == meta['sha256']
        assert meta['collision_count'] == 0 and meta['timestamps_strictly_increasing']
        with raw.open() as f: rows = list(csv.DictReader(f))
        for kind, severity, observations, labels in scenarios(rows):
            folder = Path('data/holdout_scenarios') / (raw.stem + '_' + kind + '_' + str(severity))
            folder.mkdir(parents=True, exist_ok=True)
            write_csv(folder / 'observations.csv', observations)
            write_csv(folder / 'evaluation_only.csv', [{'arrival_ns': r['arrival_ns'], 'anomaly': int(l)}
                                                      for r, l in zip(observations, labels)])
            e = load_episode(folder)
            detector = Detector('models/gru_42'); c = detector.config
            with torch.no_grad():
                x = (e['x'] - np.array(c['xm'], np.float32)) / np.array(c['xs'], np.float32)
                pred = detector.model(torch.from_numpy(x)).numpy() * c['ys'] + c['ym']
            ns = np.full(e['n'], -np.inf); ps = ns.copy()
            ns[e['indices']] = np.sqrt((( (e['y'] - pred) / c['residual_scale']) ** 2).mean(1))
            ps[e['indices']] = np.sqrt(((e['physics'] / scale) ** 2).mean(1))
            methods = {'GRU+rules': sustained((ns > c['threshold']) | e['rules'], 3),
                       'GRU_only': sustained(ns > c['threshold'], 3),
                       'rules_only': sustained(e['rules'], 3),
                       'physics+rules': sustained((ps > threshold) | e['rules'], 3),
                       'GRU_no_persistence': (ns > c['threshold']) | e['rules']}
            for method, alarm in methods.items():
                details.append(dict(episode=raw.stem, family=meta['family'], kind=kind, severity=severity,
                                    method=method, duration_s=float(e['time'][-1]),
                                    **score_events(alarm, labels, e['time'])))
            manifest.append({'episode': raw.stem, 'kind': kind, 'severity': severity,
                             'source_sha256': meta['sha256'], 'path': folder.as_posix()})
    summary = []
    for method in sorted({d['method'] for d in details}):
        clean = [d for d in details if d['method'] == method and d['kind'] == 'clean']
        clean_fp_min = sum(d['false_alarm_events'] for d in clean) / (sum(d['duration_s'] for d in clean) / 60)
        for kind, severity in sorted({(d['kind'], d['severity']) for d in details if d['kind'] != 'clean'}):
            group = [d for d in details if d['method'] == method and d['kind'] == kind and d['severity'] == severity]
            tp = sum(d['detected'] for d in group); fp = sum(d['false_alarm_events'] for d in group)
            delay = [d['delay_s'] for d in group if d['delay_s'] is not None]
            precision = tp / max(tp + fp, 1); recall = tp / len(group)
            summary.append({'method': method, 'kind': kind, 'severity': severity, 'events': len(group),
                            'detected': tp, 'recall': recall, 'precision': precision,
                            'f1': 2 * precision * recall / max(precision + recall, 1e-12),
                            'mean_detected_delay_s': float(np.mean(delay)) if delay else None,
                            'clean_false_alarms_per_min': clean_fp_min})
    write_csv(out / 'by_type.csv', summary)
    (out / 'details.json').write_text(json.dumps(details, indent=2))
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps([s for s in summary if s['method'] in ('GRU+rules', 'physics+rules')]))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, kind in zip(axes, ('smooth_position_drift', 'velocity_bias')):
        for method in ('GRU+rules', 'physics+rules', 'rules_only'):
            g = [s for s in summary if s['method'] == method and s['kind'] == kind]
            ax.plot([s['severity'] for s in g], [s['recall'] for s in g], 'o-', label=method)
        ax.set(title=kind, xlabel='Max drift slope or velocity bias (m/s)', ylabel='Event recall', ylim=(-.05, 1.05))
        ax.legend(fontsize=8); ax.grid(alpha=.2)
    fig.tight_layout(); fig.savefig(out / 'severity_recall.png', dpi=150)


if __name__ == '__main__': main()
