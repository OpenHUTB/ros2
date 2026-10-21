"""Synthetic camera-view road dataset for end-to-end steering regression.

A pinhole camera looks down a curved two-lane road. Each sample is a 48x48
grayscale camera image plus the (steering, throttle) label produced by a
pure-pursuit style controller on the same geometry. The network therefore has
to recover road curvature and lateral offset from pixels alone.
"""

from __future__ import annotations

import numpy as np

IMG = 48
HORIZON = 15
FOCAL = 46.0
CX = IMG / 2.0
LANE_W = 3.5
LOOKAHEAD = 8.0
Z_NEAR = 4.0
Z_FAR = 45.0


def _project(x_world, z):
    return CX + FOCAL * x_world / np.maximum(z, 1e-3)


def render(curvature: float, offset: float, rng, brightness: float = 1.0, noise: float = 0.02) -> np.ndarray:
    """Render the road as seen from the ego vehicle."""
    canvas = np.zeros((IMG, IMG), np.float32)
    rows = np.arange(IMG, dtype=np.float32)
    canvas += (0.30 * (1.0 - rows / IMG))[:, None]

    zs = np.linspace(Z_NEAR, Z_FAR, IMG - HORIZON)
    for idx, z in enumerate(zs):
        v = HORIZON + idx
        if v >= IMG:
            break
        centre = offset + 0.5 * curvature * z * z
        half = LANE_W / 2.0
        for lane_x, val in ((centre - half, 0.98), (centre + half, 0.98)):
            u = _project(lane_x, z)
            ui = int(round(u))
            for du, weight in ((0, 1.0), (1, 0.65), (2, 0.30)):
                uu = ui + du
                if 0 <= uu < IMG:
                    canvas[v, uu] = max(canvas[v, uu], val * weight)

    canvas *= brightness
    canvas += rng.normal(0.0, noise, canvas.shape).astype(np.float32)
    return np.clip(canvas, 0.0, 1.0)


def label(curvature: float, offset: float) -> tuple[float, float]:
    """Pure-pursuit style ground truth for (steering, throttle)."""
    steering = float(np.clip(-(curvature * LOOKAHEAD + 0.35 * offset), -1.0, 1.0))
    throttle = float(np.clip(1.0 - 0.65 * abs(steering), 0.0, 1.0))
    return steering, throttle


def make_dataset(n: int, seed: int = 0, curvature_range=(-0.004, 0.004), offset_range=(-1.2, 1.2)):
    rng = np.random.default_rng(seed)
    xs = np.zeros((n, IMG, IMG, 1), np.float32)
    ys = np.zeros((n, 2), np.float32)
    meta = np.zeros((n, 2), np.float32)
    for i in range(n):
        kappa = rng.uniform(*curvature_range)
        off = rng.uniform(*offset_range)
        xs[i, :, :, 0] = render(kappa, off, rng, brightness=rng.uniform(0.75, 1.15))
        ys[i] = label(kappa, off)
        meta[i] = (kappa, off)
    return xs, ys, meta


def road_profile(step: int, steps: int) -> tuple[float, float]:
    """Reference road used by the closed-loop demo: a smooth S-curve."""
    phase = 2.0 * np.pi * step / max(steps, 1)
    curvature = 0.0032 * np.sin(2.0 * phase)
    offset = 1.1 * np.sin(phase)
    return float(curvature), float(offset)
