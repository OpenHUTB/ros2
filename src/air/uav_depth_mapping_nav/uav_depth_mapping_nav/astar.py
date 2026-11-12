# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""栅格地图上的障碍膨胀与 8 邻域 A* 搜索。

本模块**只依赖 numpy**，不 import rclpy / 任何 ROS 消息，因此可以脱离
ROS 直接调试、跑基准与单元测试（``planner.py`` 只负责订阅/发布并调用这里）。
"""

import heapq
import math
import time

import numpy as np


def inflate(occupied, resolution, inflation_radius):
    """按半径膨胀障碍，用逐偏移平移实现，避免引入 scipy 依赖。

    膨胀的物理含义：无人机有安全半径，贴着墙飞会撞，所以把障碍向外扩一圈。
    ``occupied`` 是 (H, W) 的布尔数组，返回同形状的布尔数组。
    """
    occupied = np.asarray(occupied, dtype=bool)
    radius_cells = float(inflation_radius) / float(resolution)
    if radius_cells <= 0.0:
        return occupied.copy()

    height, width = occupied.shape
    reach = int(math.ceil(radius_cells))
    inflated = occupied.copy()
    limit = radius_cells * radius_cells
    for offset_y in range(-reach, reach + 1):
        for offset_x in range(-reach, reach + 1):
            if offset_x == 0 and offset_y == 0:
                continue
            if offset_x * offset_x + offset_y * offset_y > limit:
                continue
            shifted = np.zeros_like(occupied)
            src_y0 = max(0, -offset_y)
            src_y1 = min(height, height - offset_y)
            src_x0 = max(0, -offset_x)
            src_x1 = min(width, width - offset_x)
            if src_y0 >= src_y1 or src_x0 >= src_x1:
                continue
            shifted[src_y0 + offset_y:src_y1 + offset_y,
                    src_x0 + offset_x:src_x1 + offset_x] = \
                occupied[src_y0:src_y1, src_x0:src_x1]
            inflated |= shifted
    return inflated


def nearest_free(blocked, cell, max_ring=10):
    """单元格被占据时，在 ``max_ring`` 环内找最近的可通行格子。"""
    height, width = blocked.shape
    x, y = cell

    def is_blocked(cx, cy):
        if cx < 0 or cy < 0 or cx >= width or cy >= height:
            return True
        return bool(blocked[cy, cx])

    if not is_blocked(x, y):
        return cell
    for ring in range(1, max_ring + 1):
        for dy in range(-ring, ring + 1):
            for dx in range(-ring, ring + 1):
                if max(abs(dx), abs(dy)) != ring:
                    continue
                candidate = (x + dx, y + dy)
                if not is_blocked(*candidate):
                    return candidate
    return None


def astar(blocked, start, goal, unknown=None, allow_diagonal=True,
          allow_unknown=True, unknown_penalty=4.0, max_expansions=200000):
    """8 邻域 A*：``f(n) = g(n) + h(n)``，对角代价 sqrt(2)，检查对角穿角。

    参数
    ----
    blocked : (H, W) 布尔数组，True 表示不可通行（已膨胀）。
    start / goal : (x, y) 栅格坐标。
    unknown : (H, W) 布尔数组，True 表示该格尚未被观测到；未知格可通行但
        每步代价乘以 ``unknown_penalty``，避免为了抄近路穿过大片未知区域。
    max_expansions : 扩展上限，超过即放弃（防止大地图上卡死）。

    返回 dict：
        path          在栅格坐标系下的 (x, y) 列表；失败为空列表
        success       是否找到路径
        expansions    实际扩展的节点数
        elapsed_ms    搜索耗时（毫秒，不含障碍膨胀）
        length_cells  路径的栅格代价长度（对角按 sqrt(2) 计）
        length_m      路径的世界坐标长度（米，按 resolution 换算见调用方）
        reason        失败原因：'no_path' / 'max_expansions' / None
    """
    started = time.perf_counter()
    blocked = np.asarray(blocked, dtype=bool)
    height, width = blocked.shape

    def is_blocked(cell):
        cx, cy = cell
        if cx < 0 or cy < 0 or cx >= width or cy >= height:
            return True
        return bool(blocked[cy, cx])

    def is_unknown(cell):
        if unknown is None:
            return False
        cx, cy = cell
        return bool(unknown[cy, cx])

    result = {'path': [], 'success': False, 'expansions': 0,
              'elapsed_ms': 0.0, 'length_cells': 0.0, 'length_m': 0.0,
              'reason': None}

    if is_blocked(start) and start != goal:
        result['reason'] = 'no_path'
        return result

    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
    if allow_diagonal:
        diagonal = math.sqrt(2.0)
        moves += [(1, 1, diagonal), (1, -1, diagonal),
                  (-1, 1, diagonal), (-1, -1, diagonal)]

    def heuristic(cell):
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

    open_heap = [(heuristic(start), 0.0, start)]
    came_from = {}
    best_cost = {start: 0.0}
    closed = set()
    expansions = 0

    while open_heap:
        _, cost, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            cells = [current]
            walk = current
            while walk in came_from:
                walk = came_from[walk]
                cells.append(walk)
            cells.reverse()
            length_cells = 0.0
            for prev, nxt in zip(cells, cells[1:]):
                length_cells += math.hypot(nxt[0] - prev[0], nxt[1] - prev[1])
            result.update({
                'path': cells,
                'success': True,
                'expansions': expansions,
                'elapsed_ms': (time.perf_counter() - started) * 1000.0,
                'length_cells': length_cells,
                'reason': None,
            })
            return result
        closed.add(current)
        expansions += 1
        if expansions > max_expansions:
            result.update({'expansions': expansions,
                           'elapsed_ms': (time.perf_counter() - started) * 1000.0,
                           'reason': 'max_expansions'})
            return result

        for dx, dy, step in moves:
            neighbour = (current[0] + dx, current[1] + dy)
            if is_blocked(neighbour):
                continue
            if dx != 0 and dy != 0:
                # 对角穿角检查：两条直角边都必须是空的，否则会“擦着墙角”穿过去
                if is_blocked((current[0] + dx, current[1])):
                    continue
                if is_blocked((current[0], current[1] + dy)):
                    continue
            step_cost = step
            if allow_unknown and is_unknown(neighbour):
                step_cost *= unknown_penalty
            new_cost = cost + step_cost
            if new_cost < best_cost.get(neighbour, float('inf')):
                best_cost[neighbour] = new_cost
                came_from[neighbour] = current
                heapq.heappush(
                    open_heap, (new_cost + heuristic(neighbour), new_cost, neighbour))

    result.update({'expansions': expansions,
                   'elapsed_ms': (time.perf_counter() - started) * 1000.0,
                   'reason': 'no_path'})
    return result
