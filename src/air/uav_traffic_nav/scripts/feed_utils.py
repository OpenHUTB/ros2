#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车辆流转换的纯函数部分（不依赖 ROS，便于单元测试）."""
from __future__ import annotations

import json
import os
import time


def load_offset(path: str):
    """读 calibration.json 的 offset[0:2]；缺失或损坏返回 (0, 0)."""
    if not path or not os.path.isfile(path):
        return 0.0, 0.0
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        offset = data.get("offset", [0.0, 0.0, 0.0])
        return float(offset[0]), float(offset[1])
    except (OSError, ValueError, TypeError):
        return 0.0, 0.0


def read_live(path: str, retries: int = 5, sleep=time.sleep):
    """读 live.json；Windows 共享冲突（PermissionError）重试后返回 None."""
    for attempt in range(retries):
        try:
            with open(path, encoding="utf-8") as fh:
                return json.load(fh)
        except PermissionError:
            if attempt == retries - 1:
                return None
            sleep(0.01)
        except (OSError, ValueError):
            return None
    return None


def to_vehicles(live, offset, max_age_s: float, now=None):
    """live.json -> [{'x','y','vx','vy'}]，带坐标平移与过期过滤."""
    if not isinstance(live, dict):
        return []
    now = time.time() if now is None else now
    try:
        age = now - float(live["time"])
    except (KeyError, TypeError, ValueError):
        return []
    if age > max_age_s:
        return []
    ox, oy = offset
    out = []
    for item in live.get("vehicles", []):
        try:
            out.append({"x": float(item["x"]) + ox,
                        "y": float(item["y"]) + oy,
                        "vx": float(item.get("vx", 0.0)),
                        "vy": float(item.get("vy", 0.0))})
        except (KeyError, TypeError, ValueError):
            continue
    return out
