"""CLI for the group-crossing policy."""

from __future__ import annotations

import argparse
import json

from .group_policy import GroupCrossingPolicy


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--group-size", type=int, default=3)
    parser.add_argument("--arrived", type=int, default=2)
    parser.add_argument("--spread", type=float, default=1.0)
    parser.add_argument("--gap", type=float, default=4.0)
    parser.add_argument("--leader-waiting", action="store_true")
    parser.add_argument("--ros", action="store_true")
    args = parser.parse_args(argv)

    policy = GroupCrossingPolicy(group_size=args.group_size)
    if args.ros:
        from .node import main as node_main

        return node_main()
    decision = policy.evaluate(
        args.arrived, args.spread, args.gap, args.leader_waiting
    )
    print(json.dumps(decision.as_dict(), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
