#!/usr/bin/env python3
"""Entry point for VRU conflict-risk maps."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vru_conflict_risk_map.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
