"""CLI for takeover readiness."""

from __future__ import annotations

import argparse
import json

from .readiness import TakeoverReadinessPolicy


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gaze-on-road", action="store_true")
    parser.add_argument("--hands-on-wheel", action="store_true")
    parser.add_argument("--feet-on-pedals", action="store_true")
    parser.add_argument("--alert-age", type=float, default=1.5)
    parser.add_argument("--gaze-off", type=float, default=0.0)
    parser.add_argument("--ros", action="store_true")
    args = parser.parse_args(argv)
    if args.ros:
        from .node import main as node_main

        return node_main()
    result = TakeoverReadinessPolicy().evaluate(
        args.gaze_on_road,
        args.hands_on_wheel,
        args.feet_on_pedals,
        args.alert_age,
        args.gaze_off,
    )
    print(json.dumps(result.as_dict(), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
