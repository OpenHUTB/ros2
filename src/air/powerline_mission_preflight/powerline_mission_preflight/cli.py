"""Command-line entry point for power-line mission preflight checks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .validator import validate_mission


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate a power-line inspection mission JSON")
    parser.add_argument("mission", type=Path, help="mission JSON containing a route array")
    parser.add_argument("--report", type=Path, default=Path("preflight_report.json"))
    parser.add_argument("--min-altitude", type=float, default=5.0)
    parser.add_argument("--max-altitude", type=float, default=120.0)
    parser.add_argument("--max-leg-distance", type=float, default=80.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        mission = json.loads(args.mission.read_text(encoding="utf-8"))
        report = validate_mission(
            mission,
            min_altitude=args.min_altitude,
            max_altitude=args.max_altitude,
            max_leg_distance=args.max_leg_distance,
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise SystemExit(f"preflight input error: {exc}") from exc

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
