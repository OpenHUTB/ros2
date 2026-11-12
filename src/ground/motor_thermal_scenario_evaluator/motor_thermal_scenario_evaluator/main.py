"""Run fixed-speed and thermal-derated scenario comparisons."""

from __future__ import annotations

import argparse

from motor_thermal_scenario_evaluator.evaluate import evaluate


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare motor thermal scenarios")
    parser.add_argument("--steps", type=int, default=6000)
    parser.add_argument("--output", default="output/thermal_evaluation")
    args = parser.parse_args()
    for row in evaluate(args.steps, output=args.output):
        print(f"{row['scenario']} {row['mode']}: peak={row['peak_temperature_c']} C, mean_speed={row['mean_speed_mps']} m/s")


if __name__ == "__main__":
    main()
