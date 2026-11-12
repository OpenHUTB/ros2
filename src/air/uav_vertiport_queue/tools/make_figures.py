#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从运行结果 JSON 重新生成文档配图（不需要重跑 MuJoCo）。

用法::

    python tools/make_figures.py [--json results/vertiport_queue_result.json]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_AIR = os.path.dirname(_PKG)
_REPO = os.path.dirname(os.path.dirname(_AIR))  # <repo>/src/air/<pkg> -> <repo>
for _p in (_PKG, os.path.join(_AIR, "uav_mujoco_sim")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_vertiport_queue.core.figures import capacity_figure, wait_figure  # noqa: E402


class _Row:
    """wait_figure 需要的最小结果视图（scenario.name / metrics_*）。"""

    def __init__(self, d: dict):
        self.scenario = type("S", (), {"name": d["name"]})()
        self.metrics_before = d["before"]
        self.metrics_after = d["after"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="从结果 JSON 生成配图")
    ap.add_argument("--json", default=os.path.join(
        _PKG, "results", "vertiport_queue_result.json"))
    ap.add_argument("--out-dir", default=os.path.join(_REPO, "docs", "img", "air"),
                    help="默认直接写到文档站用的 docs/img/air，避免模块里再存一份重复图")
    args = ap.parse_args(argv)

    with open(args.json, encoding="utf-8") as fh:
        data = json.load(fh)

    cap = capacity_figure(data["capacity"]["scan"],
                          data["capacity"]["saturation"],
                          os.path.join(args.out_dir, "uav_vertiport_queue_capacity.png"))
    wait = wait_figure([_Row(d) for d in data["scenarios"]],
                       os.path.join(args.out_dir, "uav_vertiport_queue_wait.png"))
    print("已生成", cap)
    print("已生成", wait)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
