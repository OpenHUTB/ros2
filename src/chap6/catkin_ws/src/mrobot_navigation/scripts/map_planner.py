#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线地图检查与全局路径规划（不依赖 ROS / Gazebo）。

为什么需要
----------
gmapping 建完图、``map_saver`` 存完地图之后，往往要等到启动 Gazebo + move_base
才能知道这张图"能不能用"：有没有大片未知区、自由空间是不是被噪点切碎、
机器人半径下还能不能通过。这一步在真机上更麻烦——得把整套仿真再拉起来。

这个工具把 move_base 全局规划器那部分**离线**做一遍：

1. 读 ``map_server`` 格式的 ``.pgm`` + ``.yaml``（P5/P2，支持 negate 与阈值）；
2. 按 ``occupied_thresh`` / ``free_thresh`` 分成 占用 / 自由 / 未知 三类；
3. 按机器人半径做障碍膨胀（对应 costmap 的 ``inflation_radius``）；
4. A* 规划全局路径，再做视线法平滑；
5. 输出统计指标和一张 PNG 叠加图（地图 + 膨胀区 + 路径）。

只用 Python 标准库（``zlib`` 手写 PNG），所以 Windows / macOS / Ubuntu 都能直接跑，
不需要装 ROS。

用法
----
::

    python3 map_planner.py --map ../maps/gmapping_map.yaml \
        --start 1.15 5.46 --goal 9.77 5.17 --radius 0.20 --out plan.png

只做地图体检、不规划路径::

    python3 map_planner.py --map ../maps/gmapping_map.yaml --stats
"""

from __future__ import annotations

import argparse
import heapq
import math
import os
import struct
import sys
import zlib

FREE = 0
OCCUPIED = 1
UNKNOWN = 2

#: 8 邻域：(dx, dy, 步长)
_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
]


# ---------------------------------------------------------------------------
# 1. 文件读取
# ---------------------------------------------------------------------------
def parse_map_yaml(path):
    """解析 map_server 的 yaml（只认扁平的 ``key: value``，不引入 PyYAML）。

    返回 dict，包含 resolution / origin / negate / occupied_thresh /
    free_thresh 以及 ``image`` 的绝对路径。
    """
    if not os.path.isfile(path):
        raise IOError("找不到地图配置文件: %s" % path)

    values = {}
    with open(path, "r", encoding="utf-8-sig") as fh:
        for raw in fh:
            line = raw.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, val = line.split(":", 1)
            values[key.strip()] = val.strip()

    if "image" not in values:
        raise ValueError("%s 缺少 image 字段" % path)
    if "resolution" not in values:
        raise ValueError("%s 缺少 resolution 字段" % path)

    meta = {
        "image": os.path.join(os.path.dirname(os.path.abspath(path)),
                              values["image"]),
        "resolution": float(values["resolution"]),
        "origin": _parse_origin(values.get("origin", "[0, 0, 0]")),
        "negate": int(float(values.get("negate", 0))),
        "occupied_thresh": float(values.get("occupied_thresh", 0.65)),
        "free_thresh": float(values.get("free_thresh", 0.196)),
    }
    if meta["resolution"] <= 0:
        raise ValueError("resolution 必须为正")
    if meta["free_thresh"] >= meta["occupied_thresh"]:
        raise ValueError("free_thresh 必须小于 occupied_thresh")
    return meta


def _parse_origin(text):
    text = text.strip().strip("[]()")
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) < 2:
        raise ValueError("origin 至少要有 x, y 两个分量")
    return [float(x) for x in parts[:3]] if len(parts) >= 3 else \
        [float(parts[0]), float(parts[1]), 0.0]


def read_pgm(path):
    """读 PGM（P5 二进制 / P2 文本），返回 (width, height, pixels)。

    ``pixels`` 是按行展开的 0-255 灰度值，长度 = width × height。
    """
    with open(path, "rb") as fh:
        data = fh.read()

    pos = [0]

    def token():
        while pos[0] < len(data):
            ch = data[pos[0]:pos[0] + 1]
            if ch == b"#":
                while pos[0] < len(data) and data[pos[0]:pos[0] + 1] != b"\n":
                    pos[0] += 1
            elif ch.isspace():
                pos[0] += 1
            else:
                break
        start = pos[0]
        while pos[0] < len(data) and not data[pos[0]:pos[0] + 1].isspace():
            pos[0] += 1
        return data[start:pos[0]]

    magic = token()
    if magic not in (b"P5", b"P2"):
        raise ValueError("只支持 P5/P2 格式的 PGM，实际是 %r" % magic)
    width = int(token())
    height = int(token())
    maxval = int(token())
    if maxval <= 0 or maxval > 65535:
        raise ValueError("PGM 的 maxval 非法: %d" % maxval)
    if width <= 0 or height <= 0:
        raise ValueError("PGM 尺寸非法: %dx%d" % (width, height))

    if magic == b"P5":
        pos[0] += 1                      # 头部后紧跟一个空白字符
        need = width * height if maxval < 256 else width * height * 2
        body = data[pos[0]:pos[0] + need]
        if len(body) < need:
            raise ValueError("PGM 数据不完整：期望 %d 字节，实际 %d"
                             % (need, len(body)))
        if maxval < 256:
            pixels = list(body)
        else:
            pixels = [body[i] * 256 + body[i + 1] for i in range(0, need, 2)]
    else:
        pixels = [int(token()) for _ in range(width * height)]

    return width, height, pixels


def load_map(yaml_path):
    """读入地图并转成占据栅格。

    返回 ``(grid, meta)``：``grid`` 是 height×width 的二维列表，元素为
    FREE / OCCUPIED / UNKNOWN；第 0 行对应地图最上方（和 PGM 一致）。
    """
    meta = parse_map_yaml(yaml_path)
    width, height, pixels = read_pgm(meta["image"])
    if len(pixels) != width * height:
        raise ValueError("像素数与尺寸不符")

    grid = []
    for row in range(height):
        line = []
        for col in range(width):
            gray = pixels[row * width + col] / 255.0
            # map_server 的语义：先按 "白色=自由" 算占据度，negate 时反色
            p = gray if meta["negate"] else (1.0 - gray)
            if p > meta["occupied_thresh"]:
                line.append(OCCUPIED)
            elif p < meta["free_thresh"]:
                line.append(FREE)
            else:
                line.append(UNKNOWN)
        grid.append(line)

    meta["width"] = width
    meta["height"] = height
    return grid, meta


# ---------------------------------------------------------------------------
# 2. 坐标换算
# ---------------------------------------------------------------------------
def world_to_cell(x, y, meta):
    """世界坐标 (m) → 栅格 (col, row)。"""
    res = meta["resolution"]
    col = int(math.floor((x - meta["origin"][0]) / res))
    row = meta["height"] - 1 - int(math.floor((y - meta["origin"][1]) / res))
    return col, row


def cell_to_world(col, row, meta):
    """栅格 (col, row) → 世界坐标 (m)，取像素中心。"""
    res = meta["resolution"]
    x = meta["origin"][0] + (col + 0.5) * res
    y = meta["origin"][1] + (meta["height"] - 1 - row + 0.5) * res
    return x, y


def in_bounds(col, row, meta):
    return 0 <= col < meta["width"] and 0 <= row < meta["height"]


# ---------------------------------------------------------------------------
# 3. 统计
# ---------------------------------------------------------------------------
def map_statistics(grid):
    """统计三类栅格占比、最大连通自由区占比（衡量地图碎片化）。"""
    height = len(grid)
    width = len(grid[0]) if height else 0
    total = width * height
    counts = {FREE: 0, OCCUPIED: 0, UNKNOWN: 0}
    for line in grid:
        for cell in line:
            counts[cell] += 1

    largest = largest_free_component(grid)
    free_total = counts[FREE]
    return {
        "width": width,
        "height": height,
        "total": total,
        "free": counts[FREE],
        "occupied": counts[OCCUPIED],
        "unknown": counts[UNKNOWN],
        "free_ratio": free_total / float(total) if total else 0.0,
        "occupied_ratio": counts[OCCUPIED] / float(total) if total else 0.0,
        "unknown_ratio": counts[UNKNOWN] / float(total) if total else 0.0,
        "largest_component": largest,
        "largest_ratio": (largest / float(free_total)) if free_total else 0.0,
    }


def largest_free_component(grid):
    """四连通下最大的一块自由区，单位：栅格数。

    自由区被切成很多小块，通常意味着建图时噪点多、房间之间没连通，
    导航会"看得到走不过去"。
    """
    height = len(grid)
    width = len(grid[0]) if height else 0
    seen = [[False] * width for _ in range(height)]
    best = 0
    for r in range(height):
        for c in range(width):
            if grid[r][c] != FREE or seen[r][c]:
                continue
            stack = [(c, r)]
            seen[r][c] = True
            size = 0
            while stack:
                cx, cy = stack.pop()
                size += 1
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < width and 0 <= ny < height \
                            and not seen[ny][nx] and grid[ny][nx] == FREE:
                        seen[ny][nx] = True
                        stack.append((nx, ny))
            best = max(best, size)
    return best


# ---------------------------------------------------------------------------
# 4. 膨胀（chamfer 距离变换）
# ---------------------------------------------------------------------------
def distance_transform(grid):
    """到最近障碍（或未知区）的近似欧氏距离，单位：栅格。

    用两遍 chamfer 近似（水平/竖直 1、对角 √2），比逐像素画圆快得多，
    精度对膨胀半径这种量级足够。
    """
    height = len(grid)
    width = len(grid[0]) if height else 0
    inf = float("inf")
    dist = [[0.0 if grid[r][c] != FREE else inf for c in range(width)]
            for r in range(height)]

    for r in range(height):
        for c in range(width):
            if dist[r][c] == 0.0:
                continue
            best = dist[r][c]
            if r > 0:
                best = min(best, dist[r - 1][c] + 1.0)
                if c > 0:
                    best = min(best, dist[r - 1][c - 1] + math.sqrt(2.0))
                if c + 1 < width:
                    best = min(best, dist[r - 1][c + 1] + math.sqrt(2.0))
            if c > 0:
                best = min(best, dist[r][c - 1] + 1.0)
            dist[r][c] = best

    for r in range(height - 1, -1, -1):
        for c in range(width - 1, -1, -1):
            if dist[r][c] == 0.0:
                continue
            best = dist[r][c]
            if r + 1 < height:
                best = min(best, dist[r + 1][c] + 1.0)
                if c > 0:
                    best = min(best, dist[r + 1][c - 1] + math.sqrt(2.0))
                if c + 1 < width:
                    best = min(best, dist[r + 1][c + 1] + math.sqrt(2.0))
            if c + 1 < width:
                best = min(best, dist[r][c + 1] + 1.0)
            dist[r][c] = best
    return dist


def inflate(grid, radius_cells):
    """把障碍（和未知区）按半径膨胀，返回可通行掩码。

    ``mask[r][c] = True`` 表示机器人中心可以落在该栅格。
    和 costmap 的 ``inflation_radius`` 一个意思。
    """
    dist = distance_transform(grid)
    height = len(grid)
    width = len(grid[0]) if height else 0
    return [[dist[r][c] > radius_cells for c in range(width)]
            for r in range(height)]


# ---------------------------------------------------------------------------
# 5. A* 规划
# ---------------------------------------------------------------------------
def _heuristic(c1, r1, c2, r2):
    dx = abs(c1 - c2)
    dy = abs(r1 - r2)
    return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)


def plan_path(mask, start, goal):
    """在可通行掩码上做 8 邻域 A*，返回栅格路径 [(col, row), …] 或 None。

    起点/终点不可通行时返回 None；路径保证每一步都可通行。
    """
    height = len(mask)
    width = len(mask[0]) if height else 0
    sc, sr = start
    gc, gr = goal
    if not (0 <= sc < width and 0 <= sr < height):
        raise ValueError("起点超出地图范围")
    if not (0 <= gc < width and 0 <= gr < height):
        raise ValueError("终点超出地图范围")
    if not mask[sr][sc] or not mask[gr][gc]:
        return None

    open_heap = [(0.0, sc, sr)]
    g_score = {(sc, sr): 0.0}
    came = {}

    while open_heap:
        _f, c, r = heapq.heappop(open_heap)
        if (c, r) == (gc, gr):
            path = [(c, r)]
            cur = (c, r)
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path

        base = g_score[(c, r)]
        for dc, dr, step in _NEIGHBORS:
            nc, nr = c + dc, r + dr
            if not (0 <= nc < width and 0 <= nr < height):
                continue
            if not mask[nr][nc]:
                continue
            if dc and dr:                    # 斜穿墙角要拦住
                if not mask[r][nc] or not mask[nr][c]:
                    continue
            tentative = base + step
            key = (nc, nr)
            if tentative < g_score.get(key, float("inf")):
                g_score[key] = tentative
                came[key] = (c, r)
                heapq.heappush(open_heap, (tentative + _heuristic(nc, nr, gc, gr), nc, nr))
    return None


def path_length(path, resolution):
    """路径长度（米）。"""
    if not path or len(path) < 2:
        return 0.0
    total = 0.0
    for (c1, r1), (c2, r2) in zip(path[:-1], path[1:]):
        total += math.hypot(c2 - c1, r2 - r1) * resolution
    return total


def _line_cells(a, b):
    """Bresenham 直线上的所有栅格（含两端）。"""
    (c1, r1), (c2, r2) = a, b
    dc = abs(c2 - c1)
    dr = abs(r2 - r1)
    sc = 1 if c1 < c2 else -1
    sr = 1 if r1 < r2 else -1
    err = dc - dr
    c, r = c1, r1
    cells = []
    while True:
        cells.append((c, r))
        if (c, r) == (c2, r2):
            return cells
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr

def _line_clear(mask, a, b):
    """Bresenham 检查 a→b 之间是否全程可通行。"""
    (c1, r1), (c2, r2) = a, b
    dc = abs(c2 - c1)
    dr = abs(r2 - r1)
    sc = 1 if c1 < c2 else -1
    sr = 1 if r1 < r2 else -1
    err = dc - dr
    c, r = c1, r1
    while True:
        if not mask[r][c]:
            return False
        if (c, r) == (c2, r2):
            return True
        e2 = 2 * err
        nc, nr = c, r
        if e2 > -dr:
            err -= dr
            nc += sc
        if e2 < dc:
            err += dc
            nr += sr
        # 斜穿两格墙角同样要拦住，否则平滑出来的线会"切角"
        if nc != c and nr != r:
            if not mask[r][nc] or not mask[nr][c]:
                return False
        c, r = nc, nr


def smooth_path(mask, path):
    """视线法平滑：能直达就跳过中间点。

    只做几何捷径，不做曲率约束——真实差速底盘还需要 move_base 的
    local planner 再跟一遍。
    """
    if not path or len(path) < 3:
        return list(path or [])
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_clear(mask, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


# ---------------------------------------------------------------------------
# 6. PNG 输出（手写，只用 zlib）
# ---------------------------------------------------------------------------
def _png_chunk(tag, data):
    chunk = tag + data
    return (struct.pack(">I", len(data)) + chunk
            + struct.pack(">I", zlib.crc32(chunk) & 0xFFFFFFFF))


def write_png(path, width, height, rows):
    """写 8 位 RGB PNG。``rows`` 是 height 行、每行 width×3 字节。"""
    raw = bytearray()
    for row in rows:
        raw.append(0)                     # filter type 0
        raw.extend(row)
    with open(path, "wb") as fh:
        fh.write(b"\x89PNG\r\n\x1a\n")
        fh.write(_png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height,
                                                 8, 2, 0, 0, 0)))
        fh.write(_png_chunk(b"IDAT", zlib.compress(bytes(raw), 6)))
        fh.write(_png_chunk(b"IEND", b""))


_COLORS = {
    FREE: (255, 255, 255),
    OCCUPIED: (30, 30, 30),
    UNKNOWN: (190, 190, 190),
}
_INFLATED = (255, 210, 210)
_PATH = (0, 90, 220)
_START = (0, 160, 70)
_GOAL = (215, 45, 45)


def render_png(path, grid, mask, plan=None, meta=None):
    """把地图、膨胀区、路径画成 PNG。"""
    height = len(grid)
    width = len(grid[0]) if height else 0
    rows = []
    for r in range(height):
        row = bytearray()
        for c in range(width):
            if not mask[r][c] and grid[r][c] == FREE:
                row.extend(_INFLATED)
            else:
                row.extend(_COLORS[grid[r][c]])
        rows.append(row)

    def paint(cell, color, size=3):
        c0, r0 = cell
        for dr in range(-size, size + 1):
            for dc in range(-size, size + 1):
                c, r = c0 + dc, r0 + dr
                if 0 <= c < width and 0 <= r < height:
                    rows[r][3 * c:3 * c + 3] = bytes(color)

    if plan:
        # 平滑后相邻点可能隔很远，必须逐段连线，否则图上只剩几个孤点
        for a, b in zip(plan[:-1], plan[1:]):
            for cell in _line_cells(a, b):
                paint(cell, _PATH, size=1)
        paint(plan[0], _START, size=4)
        paint(plan[-1], _GOAL, size=4)

    write_png(path, width, height, rows)


# ---------------------------------------------------------------------------
# 7. 命令行入口
# ---------------------------------------------------------------------------
def describe(grid, meta):
    stats = map_statistics(grid)
    res = meta["resolution"]
    print("  地图尺寸      : %d × %d px" % (stats["width"], stats["height"]))
    print("  物理范围      : %.2f m × %.2f m（分辨率 %.3f m/px）"
          % (stats["width"] * res, stats["height"] * res, res))
    print("  原点          : [%.2f, %.2f]"
          % (meta["origin"][0], meta["origin"][1]))
    print("  自由 / 占用 / 未知 : %.1f%% / %.1f%% / %.1f%%"
          % (stats["free_ratio"] * 100, stats["occupied_ratio"] * 100,
             stats["unknown_ratio"] * 100))
    print("  最大连通自由区: %.1f%%（占自由空间）"
          % (stats["largest_ratio"] * 100))
    return stats


def main():
    ap = argparse.ArgumentParser(
        description="离线检查地图并规划全局路径（不需要 ROS）")
    ap.add_argument("--map", required=True, help="map_server 的 .yaml 文件")
    ap.add_argument("--start", nargs=2, type=float, metavar=("X", "Y"),
                    help="起点世界坐标 (m)")
    ap.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"),
                    help="终点世界坐标 (m)")
    ap.add_argument("--radius", type=float, default=0.18,
                    help="机器人半径 (m)，用于障碍膨胀，默认 0.18")
    ap.add_argument("--out", default=None, help="输出 PNG 路径")
    ap.add_argument("--stats", action="store_true", help="只输出统计")
    args = ap.parse_args()

    print()
    print("=" * 66)
    print("  离线地图检查与全局路径规划")
    print("=" * 66)

    grid, meta = load_map(args.map)
    describe(grid, meta)

    if args.stats or args.start is None or args.goal is None:
        if not args.stats:
            print("\n  提示：给出 --start X Y --goal X Y 才会规划路径")
        print()
        return 0

    radius_cells = args.radius / meta["resolution"]
    mask = inflate(grid, radius_cells)
    free_before = sum(row.count(True) for row in inflate(grid, 0.0))
    free_after = sum(row.count(True) for row in mask)
    print("  可通行面积    : %.1f%%（半径 %.2f m 膨胀后，膨胀前 %.1f%%）"
          % (100.0 * free_after / (meta["width"] * meta["height"]),
             args.radius, 100.0 * free_before / (meta["width"] * meta["height"])))

    start = world_to_cell(args.start[0], args.start[1], meta)
    goal = world_to_cell(args.goal[0], args.goal[1], meta)
    print("  起点 / 终点    : %s / %s（栅格）" % (start, goal))

    if args.out:
        out_dir = os.path.dirname(os.path.abspath(args.out))
        if out_dir and not os.path.isdir(out_dir):
            os.makedirs(out_dir)

    path = plan_path(mask, start, goal)
    if path is None:
        print("\n  ✗ 规划失败：起点或终点落在障碍/未知区，或两者不连通")
        if args.out:
            render_png(args.out, grid, mask, None, meta)
            print("  已输出地图（无路径）: %s" % args.out)
        return 2

    raw_len = path_length(path, meta["resolution"])
    smooth = smooth_path(mask, path)
    smooth_len = path_length(smooth, meta["resolution"])
    straight = math.hypot(args.goal[0] - args.start[0],
                          args.goal[1] - args.start[1])

    print("\n  规划结果")
    print("  " + "-" * 46)
    print("  A* 路径点数   : %d" % len(path))
    print("  A* 路径长度   : %.2f m" % raw_len)
    print("  平滑后点数    : %d（长度 %.2f m）" % (len(smooth), smooth_len))
    print("  直线距离      : %.2f m（绕行系数 %.2f）"
          % (straight, smooth_len / straight if straight > 0 else float("nan")))

    if args.out:
        render_png(args.out, grid, mask, smooth, meta)
        print("  叠加图已保存  : %s" % os.path.abspath(args.out))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
