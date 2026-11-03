#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""低空多机 4D 航迹冲突检测与解脱（MuJoCo 闭环验证）。"""

from .core.conflict import Conflict, Separation, detect, pair_conflicts
from .core.plan import Plan
from .core.resolve import Action, Limits, Resolution, resolve_fleet
from .core.scenarios import Scenario, get_scenario

__all__ = [
    "Conflict", "Separation", "detect", "pair_conflicts",
    "Plan", "Action", "Limits", "Resolution", "resolve_fleet",
    "Scenario", "get_scenario",
]
__version__ = "0.1.0"
