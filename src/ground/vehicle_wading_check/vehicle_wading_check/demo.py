"""Offline deterministic demo for the wading checker."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from .core import WadingChecker, WadingConfig
from .scenario import depth_at, speed_at


CSV_COLUMNS = (
    "time_s",
    "depth_m",
    "speed_mps",
    "mode",
    "buoyancy_ratio",
    "traction_limit_n",
    "drag_n",
    "v_max_mps",
    "traction_margin_n",
)


def run_demo(
    duration_s: float = 24.0,
    rate_hz: float = 10.0,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    config = WadingConfig(control_rate_hz=rate_hz)
    checker = WadingChecker(config)
    dt = 1.0 / rate_hz
    rows: list[dict[str, object]] = []
    counts = {mode: 0 for mode in ("SAFE", "CAUTION", "FLOODED", "FLOATING")}
    over_limit_samples = 0
    peak_buoyancy = 0.0
    min_v_max = float("inf")
    steps = int(round(duration_s * rate_hz)) + 1
    for index in range(steps):
        now_s = index * dt
        depth = depth_at(now_s)
        speed = speed_at(now_s)
        checker.update_depth(depth, now_s)
        checker.update_speed(speed, now_s)
        result = checker.step(now_s)
        assert result is not None
        counts[result.mode] = counts.get(result.mode, 0) + 1
        over_limit_samples += int(result.speed_mps > result.v_max_mps)
        peak_buoyancy = max(peak_buoyancy, result.buoyancy_ratio)
        min_v_max = min(min_v_max, result.v_max_mps)
        rows.append(
            {
                "time_s": round(now_s, 3),
                "depth_m": round(result.depth_m, 3),
                "speed_mps": round(result.speed_mps, 3),
                "mode": result.mode,
                "buoyancy_ratio": round(result.buoyancy_ratio, 4),
                "traction_limit_n": round(result.traction_limit_n, 1),
                "drag_n": round(result.drag_n, 1),
                "v_max_mps": round(result.v_max_mps, 3),
                "traction_margin_n": round(result.traction_margin_n, 1),
            }
        )

    depth_limit = min(
        config.intake_height_m,
        config.exhaust_height_m,
        config.door_sill_height_m,
    )
    summary = {
        "duration_s": duration_s,
        "samples": len(rows),
        "depth_limit_m": round(depth_limit, 3),
        "safe_depth_m": round(config.safety_margin_ratio * depth_limit, 3),
        "mode_counts": counts,
        "over_limit_samples": over_limit_samples,
        "peak_buoyancy_ratio": round(peak_buoyancy, 4),
        "min_v_max_mps": round(min_v_max, 3),
        "brake_decay_ratio": round(config.friction_coef_wet / config.friction_coef_dry, 4),
    }
    return rows, summary


def write_csv(rows: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def format_summary(summary: dict[str, object]) -> str:
    counts = summary["mode_counts"]
    assert isinstance(counts, dict)
    lines = [
        f"duration_s          : {summary['duration_s']}",
        f"samples             : {summary['samples']}",
        f"depth_limit_m       : {summary['depth_limit_m']} (进气/排气/车门下沿最小值)",
        f"safe_depth_m        : {summary['safe_depth_m']} (0.8 x 限深)",
        f"mode_counts         : " + ", ".join(f"{k}={v}" for k, v in counts.items()),
        f"over_limit_samples  : {summary['over_limit_samples']}",
        f"peak_buoyancy_ratio : {summary['peak_buoyancy_ratio']}",
        f"min_v_max_mps       : {summary['min_v_max_mps']}",
        f"brake_decay_ratio   : {summary['brake_decay_ratio']}",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline wading checker demo")
    parser.add_argument("--duration", type=float, default=24.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--csv", type=Path, default=None)
    args = parser.parse_args()
    rows, summary = run_demo(args.duration, args.rate)
    print(format_summary(summary))
    if args.csv is not None:
        write_csv(rows, args.csv)
        print(f"csv written to {args.csv}")


if __name__ == "__main__":
    main()
