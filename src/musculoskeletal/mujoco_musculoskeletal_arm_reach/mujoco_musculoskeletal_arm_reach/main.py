"""Module entry point for the musculoskeletal arm simulation."""

from __future__ import annotations

from .ros_node import main as run_simulation


def main() -> None:
    run_simulation()


if __name__ == '__main__':
    main()
