"""Generate the imputation corpus: flights recorded at 20 Hz with a manifest."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from . import sim

COLUMNS = ['t', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'thrust']


def generate(out_dir, n_flights=120, seed0=70000):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    manifest = []
    for i in range(n_flights):
        seed = seed0 + i
        rng = np.random.default_rng(seed)
        points = sim.random_path(rng, n_points=int(rng.integers(3, 6)),
                                 radius=float(rng.uniform(2.5, 6.0)),
                                 height=(1.5, 5.0))
        wind = float(rng.choice([0.0, 0.0, 0.15, 0.3]))
        mass_scale = float(rng.uniform(0.85, 1.25))
        data = sim.simulate(points, rng, wind_sigma=wind, mass_scale=mass_scale)
        n_train = round(0.67 * n_flights)
        n_val = round(0.17 * n_flights)
        split = 'train' if i < n_train else 'val' if i < n_train + n_val else 'test'
        name = 'flight_%d.csv' % seed
        np.savetxt(out / name, data, delimiter=',', header=','.join(COLUMNS), comments='',
                   fmt='%.6f')
        manifest.append({
            'file': name, 'seed': seed, 'split': split, 'wind_sigma': wind,
            'mass_scale': round(mass_scale, 4), 'samples': int(data.shape[0]),
            'duration_s': round(float(data[-1, 0] - data[0, 0]), 3),
            'sha256': hashlib.sha256((out / name).read_bytes()).hexdigest(),
        })
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    return manifest


def main():
    ap = argparse.ArgumentParser(description='Generate the imputation corpus')
    ap.add_argument('--output', default='flights')
    ap.add_argument('--flights', type=int, default=120)
    ap.add_argument('--seed0', type=int, default=70000)
    a = ap.parse_args()
    manifest = generate(a.output, a.flights, a.seed0)
    print(json.dumps({'flights': len(manifest),
                      'samples': sum(x['samples'] for x in manifest),
                      'output': a.output}))


if __name__ == '__main__':
    main()
