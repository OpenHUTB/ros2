"""Command-line entry point for the MuJoCo grid navigation example."""

from __future__ import annotations

import argparse
from pathlib import Path

if __package__:
    from .navigation import main as run_navigation
else:
    from navigation import main as run_navigation


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo UAV grid mapping and A* replanning")
    parser.add_argument("--output", type=Path, default=Path("output"))
    args = parser.parse_args()
    run_navigation(args.output)


if __name__ == "__main__":
    main()
