"""Verify the frozen protocol, then prepare independent final-test flights."""
import hashlib
import json
from pathlib import Path
import numpy as np
from pattern_data import CLASSES, windows


def main():
    root = Path(__file__).resolve().parent
    frozen = json.loads((root / 'results/frozen_protocol.json').read_text())
    for name, digest in frozen['sha256'].items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != digest:
            raise ValueError('Frozen file changed: ' + name)
    manifest = []
    items = []
    original = json.loads((root / 'data/raw/manifest.json').read_text())['episodes']
    original_ids = {r['episode_id'] for r in original}
    original_hashes = {r['sha256'] for r in original}
    for meta in sorted((root / 'data/final_raw').glob('episode_*.json')):
        record = json.loads(meta.read_text())
        path = meta.with_suffix('.csv')
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != record['sha256'] or digest in original_hashes or record['episode_id'] in original_ids:
            raise ValueError('Final flight checksum or split error')
        if record['collision_count'] or not record['timestamps_strictly_increasing'] or record['split'] != 'final':
            raise ValueError('Final flight quality check failed')
        chosen = windows(path)
        if not chosen:
            raise ValueError('No usable windows in ' + path.name)
        manifest.append(record)
        items.extend((x, CLASSES.index(record['family']), record['episode_id'], i) for x, i in chosen)
    if len(manifest) != 15 or any(sum(r['family'] == name for r in manifest) != 3 for name in CLASSES):
        raise ValueError('Expected 15 final flights, three per family')
    np.savez_compressed(root / 'data/prepared/final.npz', x=np.stack([r[0] for r in items]),
                        y=np.array([r[1] for r in items]), episodes=np.array([r[2] for r in items]),
                        row_indices=np.array([r[3] for r in items]))
    (root / 'data/prepared/final_manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps({'final_flights': len(manifest), 'windows': len(items),
                      'raw_states': sum(r['samples'] for r in manifest), 'frozen_hashes_verified': True}))


if __name__ == '__main__':
    main()
