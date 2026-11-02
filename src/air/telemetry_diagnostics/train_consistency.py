"""Version 2 training: supervised injection on development flights only."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset
from consistency import ConsistencyNet, extract, transform
from holdout_evaluate import scenarios


def development():
    manifest = json.loads(Path('data/prepared/manifest.json').read_text())['records']
    for r in manifest:
        if r['split'] not in ('train', 'validation') or r['kind'] != 'clean': continue
        with (Path('data/prepared') / r['path'] / 'observations.csv').open() as f:
            rows = list(csv.DictReader(f))
        raw = [dict(row, timestamp_ns=row['source_ns']) for row in rows]
        yield r['episode'], r['split'], raw
    for p in sorted(Path('data/holdout_raw').glob('*.csv')):
        with p.open() as f: raw = list(csv.DictReader(f))
        yield p.stem, 'train' if int(p.stem[-3:]) < 106 else 'validation', raw


def datasets():
    data = {'train': [], 'validation': []}; provenance = []
    for name, split, raw in development():
        provenance.append({'episode': name, 'split': split})
        # Inject each physical axis in development; raw flight grouping never changes.
        for kind, severity, rows, labels in scenarios(raw):
            if kind in ('freeze', 'dropout'): continue
            axes = range(3) if kind != 'clean' else [0]
            for axis in axes:
                changed = [r.copy() for r in rows]
                if axis:
                    key = 'p' if kind == 'smooth_position_drift' else 'v'
                    for r, original in zip(changed, raw):
                        delta = float(r[key+'x']) - float(original[key+'x'])
                        r[key+'x'] = float(original[key+'x'])
                        r[key+'xyz'[axis]] = float(original[key+'xyz'[axis]]) + delta
                x, idx, _ = extract(changed)
                data[split].append((x, labels[idx].astype(np.float32), kind == 'clean'))
    return data, provenance


def train(output, seed, variant, epochs):
    if (output / 'model.pt').exists(): raise FileExistsError('Use a new output directory')
    torch.set_num_threads(3); torch.manual_seed(seed); np.random.seed(seed)
    data, provenance = datasets()
    # Scale is fit solely to normal training windows; never fit to validation or final tests.
    clean = np.concatenate([x for x, _, normal in data['train'] if normal])
    scale = np.maximum(np.quantile(np.abs(clean[:, -1]), .95, axis=0), [.001]*3+[.01]*3).astype(np.float32)
    def make(split):
        x = np.concatenate([x for x, _, _ in data[split]])
        y = np.concatenate([y for _, y, _ in data[split]])
        return torch.from_numpy(transform(x, scale)), torch.from_numpy(y)
    tx, ty = make('train'); vx, vy = make('validation')
    loader = DataLoader(TensorDataset(tx, ty), batch_size=256, shuffle=True)
    model = ConsistencyNet(variant); optimizer = torch.optim.Adam(model.parameters(), lr=.002)
    loss_fn = torch.nn.BCEWithLogitsLoss(pos_weight=(len(ty)-ty.sum()) / ty.sum())
    best = float('inf'); log = []; output.mkdir(parents=True, exist_ok=True)
    for epoch in range(epochs):
        model.train(); total = 0
        for x, y in loader:
            optimizer.zero_grad(); loss = loss_fn(model(x), y); loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.); optimizer.step()
            total += loss.item()*len(x)
        model.eval()
        with torch.no_grad():
            val = sum(loss_fn(model(x), y).item()*len(x) for x, y in DataLoader(TensorDataset(vx, vy), batch_size=1024))/len(vx)
        log.append({'epoch': epoch+1, 'train_loss': total/len(tx), 'validation_loss': val})
        if val < best:
            best = val; torch.save(model.state_dict(), output/'model.pt')
    model.load_state_dict(torch.load(output/'model.pt', weights_only=True)); model.eval()
    normal = np.concatenate([x for x, _, normal in data['validation'] if normal])
    with torch.no_grad(): scores = model(torch.from_numpy(transform(normal, scale))).numpy()
    threshold = float(np.quantile(scores, .995))
    physical = np.sqrt(np.mean(transform(normal, scale)[:, -1]**2, axis=1))
    config = {'method':'consistency', 'variant':variant, 'seed':seed, 'history':8,
              'scale':scale.tolist(), 'threshold':threshold, 'persistence':3,
              'physics_threshold':float(np.quantile(physical,.995)), 'best_validation_bce':best,
              'selection':'validation BCE; normal validation 99.5% score quantile',
              'train_windows':len(tx), 'validation_windows':len(vx)}
    (output/'config.json').write_text(json.dumps(config,indent=2))
    (output/'loss.json').write_text(json.dumps(log,indent=2))
    (output/'development_split.json').write_text(json.dumps(provenance,indent=2))
    print(json.dumps(config),flush=True)


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,default=Path('models/retrained_consistency_42'))
    p.add_argument('--seed',type=int,default=42); p.add_argument('--variant',choices=['gru','mlp'],default='gru')
    p.add_argument('--epochs',type=int,default=20); a=p.parse_args(); train(a.output,a.seed,a.variant,a.epochs)
