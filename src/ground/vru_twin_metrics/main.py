#!/usr/bin/env python3
"""Entry point required by the OpenHUTB/ros2 module convention."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vru_twin_metrics.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
