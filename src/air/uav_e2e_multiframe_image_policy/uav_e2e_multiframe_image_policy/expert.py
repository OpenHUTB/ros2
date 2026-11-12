#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""专家参考控制器：依据横向偏移与航向误差给出参考控制指令。"""

from __future__ import annotations

import math

KP_OFFSET = 0.9
KP_HEADING = 1.1
V_BASE = 0.55
V_MAX = 0.90


def expert_command(x: float, y: float, yaw: float) -> dict:
    offset = lateral_offset(x, y)
    err = heading_error(x, yaw)
    yaw_cmd = -KP_OFFSET * offset + KP_HEADING * err
    forward = V_BASE + 0.25 * max(0.0, math.cos(err))
    forward = min(V_MAX, forward)
    return {"forward": forward, "yaw_cmd": yaw_cmd,
            "offset": offset, "heading_error": err}


# 延迟导入避免循环依赖
from .scene import lateral_offset, heading_error  # noqa: E402
