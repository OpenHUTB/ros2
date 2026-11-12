# -*- coding: utf-8 -*-
"""宽度扫描实验：4 个变体 x 7 档门宽，每档重复 REPEATS 次。"""

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

import humanoid_passage_width_sweep.config as config
import humanoid_passage_width_sweep.metrics as metrics
import humanoid_passage_width_sweep.planners as planners
from humanoid_passage_width_sweep.sim import PassageSim


@dataclass
class RunResult:
    """单次运行的完整记录。"""

    variant: str
    label: str
    width: float
    repeat: int
    row: Dict = field(default_factory=dict)
    log: Dict = field(default_factory=dict)


def run_once(variant: str, width: float, repeat: int = 0,
             duration: float = config.TIME_LIMIT, keep_log: bool = False) -> RunResult:
    """跑一次：从起点出发，直到到达 / 碰撞 / 超时。"""
    sim = PassageSim(width=width, variant=variant)
    log = sim.run(duration=duration)
    reached = (sim.goal_distance() <= config.GOAL_RADIUS) and not sim.failed
    row = metrics.summarize(log, reached, sim.failed, width, variant,
                            planners.label_of(variant), repeat)
    sim.close()
    return RunResult(variant, planners.label_of(variant), width, repeat,
                     row, log if keep_log else {})


def sweep_widths(variants=None, widths=None, repeats: int = config.REPEATS,
                 duration: float = config.TIME_LIMIT,
                 progress=None) -> List[RunResult]:
    """对每个变体、每档门宽重复跑，返回全部单次结果。"""
    variants = list(variants or planners.VARIANT_ORDER)
    widths = list(widths or config.WIDTH_GRID)
    out: List[RunResult] = []
    for v in variants:
        for w in widths:
            for r in range(repeats):
                out.append(run_once(v, w, repeat=r, duration=duration))
                if progress:
                    progress(out[-1])
    return out


def summarize(results: List[RunResult]) -> List[Dict]:
    """把单次结果聚合成「变体 x 宽度」表。"""
    return metrics.aggregate([r.row for r in results])