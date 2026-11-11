"""Create reproducible sensor observations from motor-state CSV data."""

from __future__ import annotations

import argparse

from mujoco_motor_sensor_recorder.recorder import record_csv


def main() -> None:
    parser = argparse.ArgumentParser(description="Record motor temperature and current observations")
    parser.add_argument("--input", required=True, help="CSV with motor state columns")
    parser.add_argument("--output", default="output/motor_sensors.csv")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    rows = record_csv(args.input, args.output, seed=args.seed)
    print(f"Wrote {rows} samples to {args.output}")


if __name__ == "__main__":
    main()
