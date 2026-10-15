"""Generate a synthetic LiDAR dataset for training.

This module simulates 360-beam 2D LiDAR scans without requiring a
running Gazebo simulation. Each scan is a 360-dim float32 vector where:

    * most beams hold 30.0 (the sensor's "no return" value, matching
      ``inference_node.py`` and ``collect_data.py``);
    * a few contiguous beams hold a closer distance to simulate an
      obstacle at that angle.

Labels follow the same rule used by ``collect_data.py``:

    min range <  danger_distance (1.0 m)  -> 0 (near)
    danger_distance <= min range < far_distance (3.0 m) -> 1 (far)
    min range >= far_distance             -> 2 (free)

The dataset is built so that the three classes are roughly balanced
(1/3 each) by sampling the obstacle distance from a class-conditional
uniform distribution. Gaussian noise is added to all beams so the MLP
does not memorise exact values.

Usage (no ROS 2 / Gazebo needed):

    python3 -m lidar_obstacle_nn.generate_synthetic_dataset \\
        --output /tmp/lidar_dataset.npz \\
        --num-samples 1500 \\
        --scan-size 360

Or via the entry point installed by setup.py:

    ros2 run lidar_obstacle_nn generate_synthetic_dataset --ros-args ...
"""

from __future__ import annotations

import argparse
import os
from typing import Tuple

import numpy as np

DEFAULT_SCAN_SIZE = 360
DEFAULT_DANGER_DISTANCE = 1.0
DEFAULT_FAR_DISTANCE = 3.0
DEFAULT_SCAN_MAX = 30.0
DEFAULT_NUM_SAMPLES = 1500
DEFAULT_NOISE_STD = 0.05
SEED = 42


def _simulate_scan(
    scan_size: int,
    obstacle_distance: float,
    rng: np.random.Generator,
    scan_max: float = DEFAULT_SCAN_MAX,
    noise_std: float = DEFAULT_NOISE_STD,
    obstacle_centre: int = -1,
) -> np.ndarray:
    """Simulate one 360-beam LiDAR scan.

    The obstacle is a small contiguous angular wedge. By default the
    wedge is centred at the front of the robot (beam ``scan_size // 2``
    = 180 degrees in a 360-beam scan) with a random width of 15-45
    beams. Pass ``obstacle_centre`` to override the bearing (useful
    when you want the obstacle at a random angle).

    Beams outside the wedge report ``scan_max`` (no return).
    """
    scan = np.full(scan_size, scan_max, dtype=np.float32)

    width = int(rng.integers(15, 45))
    centre = int(obstacle_centre) if obstacle_centre >= 0 else scan_size // 2
    half = width // 2
    for offset in range(-half, half + 1):
        idx = (centre + offset) % scan_size
        # Soft edges: distance fades slightly at the wedge border so
        # the network cannot memorise hard transitions.
        edge_factor = 1.0 - 0.2 * (abs(offset) / max(half, 1))
        scan[idx] = obstacle_distance * edge_factor

    # Add Gaussian noise (clipped to physical range).
    noise = rng.normal(0.0, noise_std, size=scan.shape).astype(np.float32)
    scan = np.clip(scan + noise, 0.0, scan_max)

    return scan


def label_for_min_range(
    min_range: float,
    danger_distance: float,
    far_distance: float,
) -> int:
    """Return the label id for a given minimum range."""
    if min_range < danger_distance:
        return 0
    if min_range < far_distance:
        return 1
    return 2


def generate_dataset(
    num_samples: int = DEFAULT_NUM_SAMPLES,
    scan_size: int = DEFAULT_SCAN_SIZE,
    danger_distance: float = DEFAULT_DANGER_DISTANCE,
    far_distance: float = DEFAULT_FAR_DISTANCE,
    noise_std: float = DEFAULT_NOISE_STD,
    seed: int = SEED,
    scan_max: float = DEFAULT_SCAN_MAX,
) -> Tuple[np.ndarray, np.ndarray]:
    """Generate a balanced synthetic dataset.

    Returns:
        scans  : float32 array of shape (num_samples, scan_size)
        labels : int64 array of shape (num_samples,)
    """
    rng = np.random.default_rng(seed)

    # Force class balance: split samples 1/3, 1/3, 1/3.
    counts_per_class = np.array(
        [num_samples // 3, num_samples // 3, num_samples - 2 * (num_samples // 3)]
    )
    labels = np.concatenate(
        [
            np.full(counts_per_class[0], 0, dtype=np.int64),
            np.full(counts_per_class[1], 1, dtype=np.int64),
            np.full(counts_per_class[2], 2, dtype=np.int64),
        ]
    )
    rng.shuffle(labels)

    scans = np.empty((num_samples, scan_size), dtype=np.float32)
    for i, label in enumerate(labels):
        # Sample an obstacle distance conditioned on the label.
        if label == 0:        # near: 0.1 m to 0.95 m
            obs_dist = float(rng.uniform(0.1, danger_distance - 0.05))
        elif label == 1:      # far: 1.05 m to 2.95 m
            obs_dist = float(rng.uniform(danger_distance + 0.05, far_distance - 0.05))
        else:                 # free: 3.5 m to scan_max
            obs_dist = float(rng.uniform(far_distance + 0.5, scan_max - 0.5))

        scans[i] = _simulate_scan(
            scan_size=scan_size,
            obstacle_distance=obs_dist,
            rng=rng,
            scan_max=scan_max,
            noise_std=noise_std,
        )

    return scans, labels


def save_dataset(
    output_path: str,
    scans: np.ndarray,
    labels: np.ndarray,
    scan_size: int,
    danger_distance: float,
    far_distance: float,
) -> None:
    """Save dataset to .npz with the same layout as collect_data."""
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    np.savez(
        output_path,
        scans=scans,
        labels=labels,
        scan_size=scan_size,
        danger_distance=danger_distance,
        far_distance=far_distance,
        class_names=np.asarray(['near', 'far', 'free']),
        source='synthetic',
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description='Generate a synthetic LiDAR .npz dataset for training.'
    )
    p.add_argument('--output', default='/tmp/lidar_dataset.npz')
    p.add_argument('--num-samples', type=int, default=DEFAULT_NUM_SAMPLES)
    p.add_argument('--scan-size', type=int, default=DEFAULT_SCAN_SIZE)
    p.add_argument('--danger-distance', type=float, default=DEFAULT_DANGER_DISTANCE)
    p.add_argument('--far-distance', type=float, default=DEFAULT_FAR_DISTANCE)
    p.add_argument('--noise-std', type=float, default=DEFAULT_NOISE_STD)
    p.add_argument('--seed', type=int, default=SEED)
    return p.parse_args()


def main() -> None:
    """Entry point: parse args, generate, save, report class balance."""
    args = parse_args()
    scans, labels = generate_dataset(
        num_samples=args.num_samples,
        scan_size=args.scan_size,
        danger_distance=args.danger_distance,
        far_distance=args.far_distance,
        noise_std=args.noise_std,
        seed=args.seed,
    )
    save_dataset(
        output_path=args.output,
        scans=scans,
        labels=labels,
        scan_size=args.scan_size,
        danger_distance=args.danger_distance,
        far_distance=args.far_distance,
    )

    unique, counts = np.unique(labels, return_counts=True)
    names = ['near', 'far', 'free']
    dist = ', '.join(
        '%s=%d' % (names[cid], int(cnt))
        for cid, cnt in zip(unique, counts)
    )
    print('Saved synthetic dataset: %s' % args.output)
    print('  shape: %s' % (scans.shape,))
    print('  classes: %s' % dist)


if __name__ == '__main__':
    main()
