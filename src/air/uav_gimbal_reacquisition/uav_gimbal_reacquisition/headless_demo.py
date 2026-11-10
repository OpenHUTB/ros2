"""Deterministic offline simulation entry point."""

from __future__ import annotations

import argparse
import csv
import json
import os

from .core import GimbalConfig, GimbalController, Metrics, TargetScenario
from .plotting import animate_demo, plot_pixel_error, plot_state_timeline, plot_topdown


def run_demo(duration_s: float = 50.0, dt_s: float = 0.05, config: GimbalConfig | None = None):
    scenario = TargetScenario()
    controller = GimbalController(config)
    metrics = Metrics()
    records = []
    transitions = []
    last_state = None
    steps = int(round(duration_s / dt_s))
    for index in range(steps + 1):
        time_s = index * dt_s
        sample = scenario.sample(time_s)
        observation = controller.step(sample, dt_s)
        metrics.update(observation)
        record = {
            "t": time_s,
            "target_x": sample.position[0],
            "target_y": sample.position[1],
            "target_z": sample.position[2],
            "visible": sample.visible,
            **observation.__dict__,
        }
        records.append(record)
        if observation.state != last_state:
            transitions.append(
                {
                    "t": time_s,
                    "state": observation.state,
                    "detected": observation.detected,
                    "los_error_deg": observation.los_error_deg,
                }
            )
            last_state = observation.state
    summary = metrics.summary()
    summary.update({"duration_s": duration_s, "dt_s": dt_s, "transitions": transitions})
    return records, summary


def write_csv(records, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run the gimbal loss/reacquisition demo")
    parser.add_argument("--duration", type=float, default=50.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--output-dir", default="artifacts/demo")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--no-gif", action="store_true")
    args = parser.parse_args(argv)
    records, summary = run_demo(args.duration, args.dt)
    os.makedirs(args.output_dir, exist_ok=True)
    write_csv(records, os.path.join(args.output_dir, "timeseries.csv"))
    with open(os.path.join(args.output_dir, "summary.json"), "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    if not args.no_plots:
        plot_state_timeline(records, os.path.join(args.output_dir, "gimbal_state.png"))
        plot_pixel_error(records, os.path.join(args.output_dir, "pixel_error.png"))
        plot_topdown(records, os.path.join(args.output_dir, "target_path.png"))
    if not args.no_plots and not args.no_gif:
        animate_demo(records, os.path.join(args.output_dir, "demo.gif"))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
