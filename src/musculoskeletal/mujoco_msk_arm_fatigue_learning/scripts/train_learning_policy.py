#!/usr/bin/env python3
"""Tune the online fatigue-learning gain policy."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT / 'scripts'))

from evaluate_fatigue_learning import run_episode  # noqa: E402
from mujoco_msk_arm_fatigue_learning.policy import LearningPolicy  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration-s', type=float, default=15.0)
    parser.add_argument(
        '--output',
        type=Path,
        default=(
            Path.home()
            / '.ros'
            / 'mujoco_msk_arm_fatigue_learning'
            / 'policy.json'
        ),
    )
    args = parser.parse_args()

    candidates = (
        (25.0, 0.20, 1.8),
        (50.0, 0.20, 1.8),
        (100.0, 0.20, 1.8),
        (100.0, 0.20, 2.0),
        (200.0, 0.20, 2.0),
        (100.0, 0.50, 2.0),
    )
    results = []
    fixed = run_episode(
        enable_learning=False,
        duration_s=args.duration_s,
    )
    fixed_tail = float(np.mean(fixed['error'][-200:]))
    for learning_rate, decay_rate, maximum_gain in candidates:
        episode = run_episode(
            enable_learning=True,
            duration_s=args.duration_s,
            learning_rate=learning_rate,
            gain_decay_rate=decay_rate,
            maximum_gain=maximum_gain,
        )
        tail_error = float(np.mean(episode['error'][-200:]))
        results.append(
            (
                tail_error,
                learning_rate,
                decay_rate,
                maximum_gain,
            )
        )
        print(
            'rate=%.1f decay=%.2f max_gain=%.1f tail_error=%.6f'
            % (learning_rate, decay_rate, maximum_gain, tail_error)
        )

    results.sort(key=lambda result: result[0])
    best = results[0]
    policy = LearningPolicy(
        learning_rate=best[1],
        gain_decay_rate=best[2],
        maximum_gain=best[3],
        activation_deadband=0.02,
        error_deadband_rad=0.001,
        error_ceiling_rad=0.30,
    )
    policy.save(args.output)
    print('fixed_tail_error_m=%.6f' % fixed_tail)
    print('best_tail_error_m=%.6f' % best[0])
    print('policy=%s' % args.output)


if __name__ == '__main__':
    main()
