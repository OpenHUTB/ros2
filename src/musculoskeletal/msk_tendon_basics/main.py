#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""包根入口：把 scripts/ 加到 sys.path 后转调 scripts/main.py。"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "scripts"))

from main import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
