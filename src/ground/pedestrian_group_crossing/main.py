#!/usr/bin/env python3
"""Entry point for the 2026 second-batch pedestrian group module."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pedestrian_group_crossing.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
