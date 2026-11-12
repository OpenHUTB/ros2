#!/usr/bin/env python3
"""Generate a deterministic example CSV for the standalone predictor."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def write_sample(path: str | Path, samples: int = 500, dt: float = 0.1, seed: int = 7) -> int:
    if samples < 40 or dt <= 0:
        raise ValueError("samples must be at least 40 and dt must be positive")
    rng = np.random.default_rng(seed)
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "speed_mps", "left_temperature_c", "right_temperature_c", "left_current_a", "right_current_a", "ambient_c"))
        for index in range(samples):
            time_s = index * dt
            temperature = 25.0 + 0.025 * time_s + 0.8 * np.sin(time_s / 6.0)
            current = 1.5 + 0.25 * np.sin(time_s / 2.0)
            writer.writerow((
                time_s,
                0.45 + 0.1 * np.cos(time_s / 3.0),
                temperature + rng.normal(0, 0.08),
                temperature + 0.1 + rng.normal(0, 0.08),
                current + rng.normal(0, 0.03),
                current + 0.05 + rng.normal(0, 0.03),
                24.0,
            ))
    return samples


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a reproducible predictor example CSV")
    parser.add_argument("--output", default="output/sample_temperature_series.csv")
    parser.add_argument("--samples", type=int, default=500)
    args = parser.parse_args()
    print(f"Wrote {write_sample(args.output, args.samples)} samples to {args.output}")


if __name__ == "__main__":
    main()
