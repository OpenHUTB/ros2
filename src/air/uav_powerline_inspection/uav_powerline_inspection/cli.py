# Copyright 2026 xxbzy727749

"""Command-line preview for a straight power-line inspection corridor."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .planner import build_inspection_route, estimate_route


def build_parser() -> argparse.ArgumentParser:
    """Build the route preview argument parser."""
    parser = argparse.ArgumentParser(description="Preview a UAV power-line inspection route")
    parser.add_argument("--pole-count", type=int, default=5)
    parser.add_argument("--pole-spacing", type=float, default=40.0)
    parser.add_argument("--pole-height", type=float, default=18.0)
    parser.add_argument("--bearing-deg", type=float, default=0.0)
    parser.add_argument("--standoff", type=float, default=8.0)
    parser.add_argument("--height-margin", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=4.0)
    parser.add_argument("--output", type=Path, default=Path("powerline_route.json"))
    return parser


def main(argv: list[str] | None = None) -> int:
    """Generate a preview route and return a successful process status."""
    args = build_parser().parse_args(argv)
    if args.pole_count < 2:
        raise SystemExit("pole-count must be at least 2")
    if not math.isfinite(args.pole_spacing) or args.pole_spacing <= 0:
        raise SystemExit("pole-spacing must be a positive finite number")
    angle = math.radians(args.bearing_deg)
    poles = [
        (
            index * args.pole_spacing * math.cos(angle),
            index * args.pole_spacing * math.sin(angle),
            args.pole_height,
        )
        for index in range(args.pole_count)
    ]
    route = build_inspection_route(
        poles,
        standoff=args.standoff,
        height_margin=args.height_margin,
    )
    report = {"summary": estimate_route(route, args.speed), "poles": poles, "route": route}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], indent=2))
    print(f"Preview written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
