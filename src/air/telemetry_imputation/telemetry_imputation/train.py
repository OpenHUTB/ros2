"""Train the telemetry imputer (residual over zero-order hold)."""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

from . import data, numpy_infer
from .model import Imputer


def build(root, split, seed):
    xs, ys, ms, gs = data.dataset(root, split, np.random.default_rng(seed))
    base = data.baselines(ys, ms, 'zoh')
    return xs, ys, ms, gs, base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default='flights')
    ap.add_argument('--output', default='models/imputer_42')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--epochs', type=int, default=80)
    a = ap.parse_args()

    root = Path(a.data)
    tr = build(root, 'train', a.seed + 1)
    va = build(root, 'val', a.seed + 2)
    mu, sd = data.normalizer(tr[0], tr[1], tr[2])

    def prep(part):
        xs, ys, ms, gs, base = part
        x = data.encode(xs, ms, mu, sd)
        return (torch.tensor(x), torch.tensor((ys - base) / sd), torch.tensor(ms))

    xt, yt, mt = prep(tr)
    xv, yv, mv = prep(va)

    torch.manual_seed(a.seed)
    torch.set_num_threads(2)
    net = Imputer()
    opt = torch.optim.Adam(net.parameters(), lr=2e-3)
    best, state, history = 1e9, None, []
    for epoch in range(a.epochs):
        net.train()
        perm = torch.randperm(len(xt))
        for b in perm.split(64):
            opt.zero_grad()
            out = net(xt[b])
            w = (~mt[b]).float().unsqueeze(-1)
            loss = (((out - yt[b]) ** 2) * w).sum() / w.sum().clamp(min=1) / 6
            loss.backward()
            opt.step()
        net.eval()
        with torch.no_grad():
            o = net(xv)
            wv = (~mv).float().unsqueeze(-1)
            vloss = float((((o - yv) ** 2) * wv).sum() / wv.sum().clamp(min=1) / 6)
        history.append([epoch + 1, float(loss), vloss])
        if vloss < best:
            best, state = vloss, {k: v.clone() for k, v in net.state_dict().items()}

    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    net.load_state_dict(state)
    torch.save(net.state_dict(), out / 'model.pt')
    numpy_infer.export_weights(net, out)
    (out / 'config.json').write_text(json.dumps({
        'variant': 'gru', 'hidden': 48, 'window': data.WINDOW, 'seed': a.seed,
        'mean': mu.tolist(), 'std': sd.tolist(), 'epochs': a.epochs,
        'best_val_mse': best, 'train_windows': int(len(xt)), 'val_windows': int(len(xv)),
        'parameterisation': 'residual over zero-order hold',
    }, indent=2))
    np.savetxt(out / 'loss.csv', np.asarray(history), delimiter=',',
               header='epoch,train_mse,val_mse', comments='')
    print(json.dumps({'output': str(out), 'best_val_mse': best,
                      'train_windows': int(len(xt)), 'epochs': a.epochs}))


if __name__ == '__main__':
    main()
