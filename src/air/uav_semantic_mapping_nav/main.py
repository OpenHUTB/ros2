#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_semantic_mapping_nav 统一入口。逻辑在 `uav_semantic_mapping_nav/cli.py`。"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from uav_semantic_mapping_nav.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
