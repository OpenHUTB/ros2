#!/usr/bin/env python3
"""Entry point for driver takeover readiness."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from driver_takeover_readiness.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
