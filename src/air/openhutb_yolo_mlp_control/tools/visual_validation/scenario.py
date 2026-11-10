"""Shared timing and file protocol; no simulator or ROS imports."""
import json
import math
import os
from pathlib import Path

DURATION = 55.0


def phase(elapsed):
    """A is hidden briefly and then for longer than the lock timeout."""
    return {'hide_a': 15 < elapsed < 16 or 30 < elapsed < 35,
            'hide_b': elapsed < 5,
            'offset_a': .35 * math.sin(elapsed * .1)}


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2), encoding='utf-8')
    os.replace(temporary, path)


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def output_directory(value):
    path = Path(value).expanduser().resolve()
    if any((p / '.git').exists() for p in (path, *path.parents)):
        raise ValueError('Use an output directory outside any Git checkout')
    path.mkdir(parents=True, exist_ok=True)
    return path


def summarize(rows, observations, ready):
    """Report evidence, never infer identity solely from a nonzero car count."""
    guards = [r for r in rows if r['status'] not in ('tracking', 'centered')]
    intervals = {}
    for name, low, high in [('two_visible', 6, 14), ('a_hidden', 31, 34),
                            ('returned_waiting', 36, 44)]:
        frames = [o for o in observations if low <= o['t'] - ready <= high]
        intervals[name] = {'frames': len(frames), 'car_counts': [
            sum(b[5] == 'car' for b in o['boxes']) for o in frames]}
    return {'guard_nonzero': sum(any(abs(v) > 1e-9 for v in r['velocity_ned'])
                                 for r in guards),
            'intervals': intervals,
            'note': 'Inspect boxes, association diagnostics and scene records; '
                    'these counts alone do not prove vehicle identity.'}
