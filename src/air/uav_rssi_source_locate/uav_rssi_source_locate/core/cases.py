#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Beacon layouts used by the CLI, the results and the docs.

Single-line summary: four hidden-beacon scenarios — where the source sits
relative to the search area decides how much of the first detection ring
carries signal, which is exactly what the mirror ambiguity feeds on.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple

from .locator import SearchCfg


@dataclass(frozen=True)
class Scenario:
    """一个隐藏信标算例。"""

    name: str
    beacon_xy: Tuple[float, float]
    note: str
    cfg: SearchCfg = field(default_factory=SearchCfg)


SCENARIOS: Dict[str, Scenario] = {
    "center": Scenario(
        "center", (140.0, 90.0), "区内中心：首圈即整圈有信号"),
    "southeast": Scenario(
        "southeast", (200.0, 20.0), "东南角：首圈弧段偏短"),
    "northwest": Scenario(
        "northwest", (30.0, 140.0), "西北角：首圈只剩单侧弧，镜像歧义最重"),
    "east_edge": Scenario(
        "east_edge", (215.0, 150.0), "东北边：信标贴近区界"),
}

CASES = tuple(SCENARIOS)


def make_scenario(name: str) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError("unknown scenario %r (have %s)"
                       % (name, ", ".join(SCENARIOS)))
    return SCENARIOS[name]
