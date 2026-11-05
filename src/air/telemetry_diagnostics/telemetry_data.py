"""Deterministic observation-layer injection. No simulator ground truth in inputs."""
import argparse
import csv
import hashlib
import json
import random
from pathlib import Path

CHANNELS = ['px', 'py', 'pz', 'vx', 'vy', 'vz', 'ax', 'ay', 'az',
            'qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz']
KINDS = ['clean', 'position_drift', 'velocity_spike', 'freeze', 'dropout']


def inject(rows, kind, seed, severity):
    """Returns separate observations and evaluation-only labels.

    Dropouts remain scheduled events with delivered=0, never zero-valued packets.
    Freeze repeats both payload and source timestamp while arrival time advances.
    """
    if kind not in KINDS or len(rows) < 40:
        raise ValueError('Unknown scenario or insufficient episode length')
    rng = random.Random(seed)
    start = rng.randrange(len(rows) // 3, len(rows) // 2)
    end = min(len(rows) - 10, start + len(rows) // 4)
    axis = rng.choice('xyz')
    sign = rng.choice([-1, 1])
    observations, labels = [], []
    onset = int(rows[start]['timestamp_ns'])
    for i, row in enumerate(rows):
        active = kind != 'clean' and start <= i < end
        source = rows[start - 1] if active and kind == 'freeze' else row
        delivered = not (active and kind == 'dropout')
        obs = {'arrival_ns': int(row['timestamp_ns']),
               'source_ns': int(source['timestamp_ns']) if delivered else '',
               'delivered': int(delivered)}
        obs.update({k: float(source[k]) if delivered else '' for k in CHANNELS})
        if active and kind == 'position_drift':
            obs['p' + axis] += sign * severity * (int(row['timestamp_ns']) - onset) / 1e9
        if active and kind == 'velocity_spike':
            obs['v' + axis] += sign * severity
        observations.append(obs)
        labels.append({'arrival_ns': obs['arrival_ns'], 'anomaly': int(active),
                       'kind': kind if active else 'clean',
                       **{'reference_' + k: row[k] for k in CHANNELS}})
    return observations, labels


def write_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def prepare(source, output):
    original = json.loads((source / 'processed/manifest.json').read_text())
    records, seen = [], set()
    for n, item in enumerate(original['episodes']):
        episode, split = item['episode_id'], item['split']
        if episode in seen:
            raise ValueError('Source episode assigned more than once')
        seen.add(episode)
        path = source / 'raw' / (episode + '.csv')
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != item['sha256']:
            raise ValueError('Raw data checksum mismatch: ' + episode)
        with path.open(newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
        times = [int(r['timestamp_ns']) for r in rows]
        if not all(b > a for a, b in zip(times, times[1:])):
            raise ValueError('Non-increasing timestamp')
        # Healthy-only training; validation calibrates thresholds, test stays held out.
        kinds = ['clean'] if split == 'train' else KINDS
        for kind in kinds:
            severity = 0.3 if split != 'test' else 0.6
            seed = 20260930 + n
            obs, labels = inject(rows, kind, seed, severity)
            folder = output / split / (episode + '_' + kind)
            folder.mkdir(parents=True, exist_ok=True)
            write_csv(folder / 'observations.csv', obs)
            write_csv(folder / 'evaluation_only.csv', labels)
            records.append({'episode': episode, 'split': split, 'kind': kind,
                            'family': item['family'], 'source_sha256': digest,
                            'seed': seed, 'severity': severity, 'rows': len(rows),
                            'path': folder.relative_to(output).as_posix()})
    (output / 'manifest.json').write_text(json.dumps({
        'description': 'Real AirSim trajectories with software-injected telemetry anomalies',
        'timestamp_semantics': 'arrival_ns uses recorded simulator time as replay clock',
        'drift_units': 'm/s', 'velocity_spike_units': 'm/s',
        'training': 'clean only', 'records': records}, indent=2), encoding='utf-8')
    return records


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, default=Path('data/prepared'))
    a = p.parse_args()
    records = prepare(a.source, a.output)
    print(json.dumps({'scenarios': len(records), 'rows': sum(r['rows'] for r in records)}))
