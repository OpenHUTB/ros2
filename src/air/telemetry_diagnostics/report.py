"""Render recorded experiments; no fabricated curves or metrics."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stream_detector import Detector


def main():
    out = Path('results'); out.mkdir(exist_ok=True)
    summaries = []
    fig, ax = plt.subplots(figsize=(8, 4))
    for folder in sorted(Path('models').glob('*')):
        if folder.name not in ('gru_42', 'gru_43', 'gru_44', 'mlp_42'): continue
        if not (folder / 'metrics.json').exists(): continue
        result = json.loads((folder / 'metrics.json').read_text())
        summary = {k: v for k, v in result.items() if k not in ('details', 'without_persistence', 'physics_baseline')}
        summary['model'] = folder.name; summaries.append(summary)
        history = json.loads((folder / 'loss.json').read_text())
        ax.plot([h['epoch'] for h in history], [h['validation_loss'] for h in history], label=folder.name)
    ax.set(xlabel='Epoch', ylabel='Normalized validation MSE', title='Actual training records')
    ax.legend(); ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(out / 'loss.png', dpi=160); plt.close(fig)
    with (out / 'metrics.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    records = json.loads(Path('data/prepared/manifest.json').read_text())['records']
    fig, axes = plt.subplots(4, 1, figsize=(10, 10))
    for ax, kind in zip(axes, ['position_drift', 'velocity_spike', 'freeze', 'dropout']):
        record = next(r for r in records if r['split'] == 'test' and r['kind'] == kind)
        path = Path('data/prepared') / record['path']
        with (path / 'observations.csv').open() as f: rows = list(csv.DictReader(f))
        with (path / 'evaluation_only.csv').open() as f: labels = list(csv.DictReader(f))
        detector = Detector('models/gru_42')
        results = [detector.step(r) for r in rows]
        t = np.array([(int(r['arrival_ns']) - int(rows[0]['arrival_ns'])) / 1e9 for r in rows])
        anomaly = np.array([int(r['anomaly']) for r in labels], dtype=bool)
        ax.plot(t, [r['score'] if r['score'] is not None else np.nan for r in results], label='Neural score')
        ax.axhline(detector.config['threshold'], color='orange', ls='--', label='Validation threshold')
        ax.axvspan(t[anomaly][0], t[anomaly][-1], color='red', alpha=.12, label='Injected interval (evaluation only)')
        alarms = np.array([r['alarm'] for r in results])
        ax.scatter(t[alarms], np.zeros(alarms.sum()), color='red', marker='|', label='Hybrid alarm')
        ax.set(title=kind, xlabel='Simulation seconds', ylabel='Score'); ax.grid(alpha=.2)
    axes[0].legend(fontsize=8); fig.tight_layout(); fig.savefig(out / 'detection_examples.png', dpi=150); plt.close(fig)
    first = json.loads(Path('models/gru_42/metrics.json').read_text())
    items = [(s['model'], s['event_f1']) for s in summaries]
    items += [('physics+rules', first['physics_baseline']['event_f1']),
              ('GRU42 no persistence', first['without_persistence']['event_f1'])]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar([i[0] for i in items], [i[1] for i in items]); ax.set(ylabel='Event F1', ylim=(0, 1.05))
    ax.tick_params(axis='x', rotation=25); fig.tight_layout(); fig.savefig(out / 'ablation.png', dpi=150); plt.close(fig)
    print(json.dumps(summaries))


if __name__ == '__main__': main()
