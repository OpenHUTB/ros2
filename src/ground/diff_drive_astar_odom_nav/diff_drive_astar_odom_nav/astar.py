#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""栅格 A* 全局路径规划核心（不含任何 ROS 依赖，便于单元测试）。

坐标约定：grid[row, col]，row 沿 y 增大，col 沿 x 增大；8 邻接，
直行代价 1、对角代价 sqrt(2)，启发式取 octile 距离，因而 w=1 时最优。
"""

import heapq
import math

import numpy as np

SQRT2 = math.sqrt(2.0)


def inflate_grid(grid, radius_cells):
    """按机器人半径膨胀障碍（切比雪夫/方形结构元）。

    用横向、纵向两次「取最大值」的可分离滤波实现，复杂度 O(HW)，
    与膨胀半径无关。膨胀把机器人当作圆，保证路径中心离障碍至少一个半径。
    """
    if radius_cells <= 0:
        return grid.copy()
    r = int(radius_cells)
    height, width = grid.shape
    tmp = grid.copy()
    for offset in range(1, r + 1):
        tmp[:, offset:] |= grid[:, :-offset]
        tmp[:, :-offset] |= grid[:, offset:]
    out = tmp.copy()
    for offset in range(1, r + 1):
        out[offset:, :] |= tmp[:-offset, :]
        out[:-offset, :] |= tmp[offset:, :]
    return out


def _nearest_free(grid, cell):
    """BFS 找最近的空闲格；找不到返回 None。"""
    height, width = grid.shape
    row, col = cell
    if 0 <= row < height and 0 <= col < width and not grid[row, col]:
        return cell
    seen = {(row, col)}
    queue = [(row, col)]
    while queue:
        row, col = queue.pop(0)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (row + dr, col + dc)
            if not (0 <= nxt[0] < height and 0 <= nxt[1] < width):
                continue
            if nxt in seen:
                continue
            if not grid[nxt]:
                return nxt
            seen.add(nxt)
            queue.append(nxt)
    return None


def is_connected(grid, start, goal):
    """4-邻接连通性预检，避免对不连通的目标做无谓搜索。"""
    height, width = grid.shape
    if not (0 <= start[0] < height and 0 <= start[1] < width):
        return False
    if not (0 <= goal[0] < height and 0 <= goal[1] < width):
        return False
    if grid[start] or grid[goal]:
        return False
    stack = [start]
    seen = {start}
    while stack:
        row, col = stack.pop()
        if (row, col) == goal:
            return True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (row + dr, col + dc)
            if 0 <= nxt[0] < height and 0 <= nxt[1] < width \
                    and not grid[nxt] and nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    return False


def find_path(grid, start, goal, allow_diagonal=True, weight=1.0,
              max_expansions=200000):
    """在占用栅格上搜索从 start 到 goal 的最短路径。

    参数均为 (row, col) 整数格。返回 (path, info)：
      path 为格点列表（含起终点），无解时为空列表；
      info 含 found / expanded / cost / reason。
    """
    height, width = grid.shape
    info = {"found": False, "expanded": 0, "cost": 0.0, "reason": ""}

    if not (0 <= start[0] < height and 0 <= start[1] < width):
        info["reason"] = "start out of map"
        return [], info
    if not (0 <= goal[0] < height and 0 <= goal[1] < width):
        info["reason"] = "goal out of map"
        return [], info

    if grid[start]:
        fixed = _nearest_free(grid, start)
        if fixed is None:
            info["reason"] = "start in obstacle and no free cell"
            return [], info
        start = fixed
    if grid[goal]:
        fixed = _nearest_free(grid, goal)
        if fixed is None:
            info["reason"] = "goal in obstacle and no free cell"
            return [], info
        goal = fixed

    if not is_connected(grid, start, goal):
        info["reason"] = "start and goal are not connected"
        return [], info

    if allow_diagonal:
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]

        def heuristic(a, b):
            dr = abs(a[0] - b[0])
            dc = abs(a[1] - b[1])
            return SQRT2 * min(dr, dc) + (max(dr, dc) - min(dr, dc))
    else:
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_heap = [(weight * heuristic(start, goal), 0.0, start)]
    g_score = {start: 0.0}
    parent = {start: None}
    closed = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        info["expanded"] += 1
        if current == goal:
            info["found"] = True
            info["cost"] = g
            break
        if info["expanded"] >= max_expansions:
            info["reason"] = "max_expansions reached"
            break

        for dr, dc, step in moves:
            nxt = (current[0] + dr, current[1] + dc)
            if not (0 <= nxt[0] < height and 0 <= nxt[1] < width) or grid[nxt]:
                continue
            # 严格穿角：对角移动要求两侧格都是空闲，避免路径擦过障碍角
            if dr != 0 and dc != 0:
                if grid[current[0] + dr, current[1]] or grid[current[0], current[1] + dc]:
                    continue
            tentative = g + step
            if tentative < g_score.get(nxt, float("inf")):
                g_score[nxt] = tentative
                parent[nxt] = current
                # f 相同时 h 小者优先，减少开阔地形上的无谓扩展
                f = tentative + weight * heuristic(nxt, goal)
                heapq.heappush(open_heap, (f, tentative, nxt))

    if not info["found"]:
        if not info["reason"]:
            info["reason"] = "no path found"
        return [], info

    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path, info


def simplify_collinear(path):
    """去掉共线的中间格，减少路径点数（不改变几何形状）。"""
    if len(path) < 2:
        return list(path)
    out = [path[0]]
    for i in range(1, len(path) - 1):
        prev_dir = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        next_dir = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if prev_dir != next_dir:
            out.append(path[i])
    out.append(path[-1])
    return out


def path_length_world(points):
    """折线长度（点已是米制坐标）。"""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total
