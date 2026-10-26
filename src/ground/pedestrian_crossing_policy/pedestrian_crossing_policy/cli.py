"""Command-line entry point for the crossing policy."""

from __future__ import annotations

import argparse

from .policy import CrossingDecision, CrossingPolicy


def _print_decision(distance: float, closing_speed: float, policy: CrossingPolicy) -> int:
    decision = CrossingDecision.evaluate(policy, distance, closing_speed)
    ttc = "inf" if decision.time_to_collision is None else "{:.3f}s".format(decision.time_to_collision)
    print("wait={} reason={} ttc={}".format(decision.wait, decision.reason, ttc))
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=float, help="vehicle-to-pedestrian distance (m)")
    parser.add_argument("--closing-speed", type=float, default=0.0, help="approach speed (m/s)")
    parser.add_argument("--min-ttc", type=float, default=3.0)
    parser.add_argument("--max-distance", type=float, default=12.0)
    parser.add_argument("--min-distance", type=float, default=1.5)
    parser.add_argument("--ros", action="store_true", help="run the ROS 2 node")
    args = parser.parse_args(argv)

    policy = CrossingPolicy(
        min_time_to_collision=args.min_ttc,
        max_vehicle_distance=args.max_distance,
        min_vehicle_distance=args.min_distance,
    )
    policy.validate()

    if args.ros:
        from .node import main as node_main

        return node_main()
    if args.distance is None:
        print("self-test: approaching vehicle inside TTC should wait")
        return _print_decision(10.0, 10.0, policy)
    return _print_decision(args.distance, args.closing_speed, policy)


if __name__ == "__main__":
    raise SystemExit(main())
