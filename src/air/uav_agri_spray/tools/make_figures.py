#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regenerate docs figures from results/uav_agri_spray_result.json."""
from __future__ import annotations

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
for _p in (_PKG_ROOT, os.path.normpath(os.path.join(_PKG_ROOT, "..",
                                                    "uav_mujoco_sim"))):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_agri_spray import viz  # noqa: E402


def main() -> int:
    results = os.path.join(_PKG_ROOT, "results", "uav_agri_spray_result.json")
    images = os.path.join(_PKG_ROOT, "images")
    with open(results, encoding="utf-8") as f:
        report = json.load(f)
    outs = viz.write_maps(report, images)
    for o in outs:
        print("wrote %s" % o)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
