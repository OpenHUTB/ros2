#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地形高度场与可通行性基准。

地面为零高度基准，左右两侧带状区域为抬高的台阶障碍，夹出中央通行
走廊；深度传感器与通行度网络都以本地高度差（台阶/坡度）作为感知输入。
"""

from __future__ import annotations

X_MIN, X_MAX = 0.0, 12.0
Y_MIN, Y_MAX = -4.0, 4.0

BLOCK_X0, BLOCK_X1 = 3.0, 7.0
BLOCK_Y0, BLOCK_Y1 = 0.5, 2.2      # 左侧抬高障碍
RIGHT_Y0, RIGHT_Y1 = -2.2, -0.5     # 右侧抬高障碍
BLOCK_HEIGHT = 0.35
STEP_LIMIT = 0.12      # 超过该台阶高度判定为不可通行
SLOPE_LIMIT = 0.35     # 坡度阈值


def _in_block(x: float, y: float) -> bool:
    if not (BLOCK_X0 <= x <= BLOCK_X1):
        return False
    left = BLOCK_Y0 <= y <= BLOCK_Y1
    right = RIGHT_Y0 <= y <= RIGHT_Y1
    return left or right


def height(x: float, y: float) -> float:
    """地形高度 m，两侧抬高障碍抬高。"""
    return BLOCK_HEIGHT if _in_block(x, y) else 0.0


def step_height(x: float, y: float, ds: float = 0.25) -> float:
    """本地最大台阶高度：与四个邻域的高度差最大值。"""
    h0 = height(x, y)
    neighbors = [
        height(x + ds, y), height(x - ds, y),
        height(x, y + ds), height(x, y - ds),
    ]
    return max(abs(h0 - hn) for hn in neighbors)


def slope(x: float, y: float, ds: float = 0.25) -> float:
    """本地坡度近似值（高度差 / 水平距离）。"""
    return step_height(x, y, ds) / ds


def is_blocked(x: float, y: float) -> bool:
    """基准可通行性：落在抬高障碍上即不可通行。"""
    return height(x, y) >= BLOCK_HEIGHT * 0.5
