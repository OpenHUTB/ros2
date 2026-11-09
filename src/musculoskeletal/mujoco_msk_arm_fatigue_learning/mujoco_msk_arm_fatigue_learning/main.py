"""Module entry point for the fatigue-learning node."""

from __future__ import annotations

from .ros_node import main as run_node


def main() -> None:
    run_node()


if __name__ == '__main__':
    main()
