#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从运行结果 JSON 重新生成文档用图（images/footprint*.png）。

不重跑优化与闭环，只读 results/*.json 里的航迹/敏感点/范围，把足迹场
重算一遍后绘图——保证图与 JSON 完全一致。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)          # 模块根目录
_AIR = os.path.dirname(_ROOT)
for _p in (_ROOT, os.path.join(_AIR, "uav_mujoco_sim")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_noise_footprint.core.pipeline import CaseResult  # noqa: E402
from uav_noise_footprint.core.scenarios import get_scenario  # noqa: E402
from uav_noise_footprint.core.track import Flight  # noqa: E402
from uav_noise_footprint.figures import draw_footprint_all  # noqa: E402


def results_from_json(path: str) -> list:
    """把 results JSON 还原成 CaseResult 列表（只用于绘图）。"""
    with open(path, encoding="utf-8") as fh:
        payload = json.load(fh)
    out = []
    for s in payload:
        sc = get_scenario(s["scenario"])
        out.append(CaseResult(
            scenario=sc,
            baseline=[Flight.from_dict(d) for d in s["baseline_flights"]],
            resolution=None,      # 绘图不需要
            summary=s))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", default=os.path.join(_ROOT, "results",
                                                   "all_result.json"))
    ap.add_argument("--out", default=os.path.join(_ROOT, "images"))
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    path = draw_footprint_all(results_from_json(args.json), args.out)
    print("图已生成：%s" % path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
