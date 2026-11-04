#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""屋顶感知：从 LiDAR 点云中检测水平屋顶平面并选取安全降落点。

算法流程：

1. **RANSAC 拟合平面** —— 抗离群点，并在内点上做最小二乘精修；
2. **水平性筛选** —— 只保留法向与竖直方向夹角 <= ``max_tilt_deg`` 的平面
   （屋顶、地面都属于水平面，靠高度与面积区分）；
3. **去掉地面** —— 默认按 ``min_height_mode="relative"``：以检出的最低水平面
   （地面）为基准，丢掉与它高度差小于 ``ground_margin`` 的平面，因此**不要求
   地图原点落在地面高度上**（换地图实测过的坑，详见 :func:`detect_rooftops`）；
4. **屋顶轮廓** —— 把平面内点投影到水平面，取 2D 凸包；
5. **降落点** —— 用 polylabel（**最大内切圆圆心**）求「离屋顶边缘最远」的点，
   这正是安全降落点：四周余量最大，最不容易撞到女儿墙或掉下去；
6. **打分选优** —— 综合内切圆半径、面积、高度（越高越像屋顶而不是地面）排序。

本模块只有 numpy 依赖、不 ``import rospy``，因此可以在没有 ROS / 仿真器的
机器上做完整回归测试，见仓库 ``tests/test_rooftop_landing_local.py``。
"""
from __future__ import annotations

import math

import numpy as np

_SQRT2 = math.sqrt(2.0)


# --------------------------------------------------------------------------- #
# 1. 平面拟合
# --------------------------------------------------------------------------- #
def fit_plane_ransac(points, dist_thresh=0.15, iters=300, min_inliers=50,
                     rng=None):
    """RANSAC 拟合平面，返回 dict 或 None。

    平面用 ``normal · p + d = 0`` 表示，法向统一朝上（``normal[2] >= 0``）。

    返回键：
        normal   (3,) 单位法向
        d        float 平面偏移
        inliers  (N,) bool 内点掩码
        count    int 内点数
        centroid (3,) 内点质心
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) < 3:
        return None

    rng = rng if rng is not None else np.random.default_rng(0)
    n_pts = len(pts)
    sample = min(n_pts, max(60, min(400, n_pts)))
    pool = pts[rng.choice(n_pts, sample, replace=False)] if n_pts > sample else pts

    best_count, best_params = -1, None
    for _ in range(int(iters)):
        idx = rng.choice(len(pool), 3, replace=False)
        p0, p1, p2 = pool[idx]
        n = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(n))
        if norm < 1e-9:
            continue
        n = n / norm
        d = -float(np.dot(n, p0))
        count = int((np.abs(pts @ n + d) < dist_thresh).sum())
        if count > best_count:
            best_count, best_params = count, (n, d)
    if best_params is None or best_count < min_inliers:
        return None

    n, d = best_params
    for _ in range(2):  # 用内点做最小二乘精修（迭代两次即可收敛）
        inliers = np.abs(pts @ n + d) < dist_thresh
        if int(inliers.sum()) < 3:
            return None
        in_pts = pts[inliers]
        centroid = in_pts.mean(axis=0)
        # 法向 = 最小奇异值方向
        _, _, vt = np.linalg.svd(in_pts - centroid, full_matrices=False)
        n = vt[-1]
        if n[2] < 0:
            n = -n
        d = -float(np.dot(n, centroid))

    inliers = np.abs(pts @ n + d) < dist_thresh
    count = int(inliers.sum())
    if count < min_inliers:
        return None
    return {"normal": n, "d": d, "inliers": inliers, "count": count,
            "centroid": pts[inliers].mean(axis=0)}


def plane_tilt_deg(normal):
    """平面法向与竖直方向的夹角（度）。0 表示完全水平。"""
    n = np.asarray(normal, dtype=np.float64)
    norm = float(np.linalg.norm(n))
    if norm < 1e-12:
        return 90.0
    cos_tilt = abs(float(n[2])) / norm
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))


def plane_height(normal, d):
    """平面在竖直方向上的高度：满足 ``normal·p + d = 0`` 且 p=(0,0,z) 的 z。

    水平面法向为 ``(0,0,1)`` 时，高度就是 ``-d``。
    """
    nz = float(normal[2])
    if abs(nz) < 1e-9:
        return float("nan")
    return -float(d) / nz


# --------------------------------------------------------------------------- #
# 2. 2D 多边形工具
# --------------------------------------------------------------------------- #
def convex_hull_2d(points):
    """Andrew 单调链求凸包，返回逆时针排列的 ``(M, 2)`` 顶点（M>=3 才有意义）。"""
    pts = np.asarray(points, dtype=np.float64)[:, :2]
    if len(pts) < 3:
        return np.zeros((0, 2), dtype=np.float64)
    order = np.lexsort((pts[:, 1], pts[:, 0]))
    pts = pts[order]
    # 去掉重复点
    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = np.any(np.abs(np.diff(pts, axis=0)) > 1e-12, axis=1)
    pts = pts[keep]
    if len(pts) < 3:
        return np.zeros((0, 2), dtype=np.float64)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = np.array(lower[:-1] + upper[:-1], dtype=np.float64)
    return hull if len(hull) >= 3 else np.zeros((0, 2), dtype=np.float64)


def polygon_area(poly):
    """鞋带公式求多边形面积（绝对值）。"""
    pts = np.asarray(poly, dtype=np.float64)[:, :2]
    if len(pts) < 3:
        return 0.0
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _seg_dist_sq(px, py, a, b):
    x, y = float(a[0]), float(a[1])
    dx, dy = float(b[0]) - x, float(b[1]) - y
    if dx != 0.0 or dy != 0.0:
        t = ((px - x) * dx + (py - y) * dy) / (dx * dx + dy * dy)
        if t > 1.0:
            x, y = float(b[0]), float(b[1])
        elif t > 0.0:
            x += dx * t
            y += dy * t
    dx, dy = px - x, py - y
    return dx * dx + dy * dy


def point_to_polygon_distance(x, y, rings):
    """点到多边形（外环 + 若干内环）的有符号距离，内部为正。"""
    inside = False
    min_dist_sq = float("inf")
    for ring in rings:
        ring = np.asarray(ring, dtype=np.float64)
        if len(ring) < 3:
            continue
        b = ring[-1]
        for a in ring:
            if ((a[1] > y) != (b[1] > y)) and (
                    x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]):
                inside = not inside
            d = _seg_dist_sq(x, y, a, b)
            if d < min_dist_sq:
                min_dist_sq = d
            b = a
    if min_dist_sq == float("inf"):
        return 0.0
    res = math.sqrt(min_dist_sq)
    return res if inside else -res


class _Cell(object):
    __slots__ = ("x", "y", "h", "d", "max")

    def __init__(self, x, y, h, d):
        self.x, self.y, self.h, self.d = x, y, h, d
        self.max = d + h * _SQRT2


def polylabel(rings, precision=0.05):
    """Mapbox polylabel：求多边形的**最大内切圆圆心**（最远离边界的内点）。

    返回 ``((x, y), radius)``；输入退化时返回 ``((0, 0), 0.0)``。
    """
    rings = [np.asarray(r, dtype=np.float64)[:, :2] for r in rings
             if len(np.asarray(r)) >= 3]
    if not rings:
        return (0.0, 0.0), 0.0
    all_pts = np.vstack(rings)
    minx, miny = all_pts.min(axis=0)
    maxx, maxy = all_pts.max(axis=0)
    width, height = maxx - minx, maxy - miny
    cell_size = min(width, height)
    if not math.isfinite(cell_size) or cell_size <= 0.0:
        return (float(minx), float(miny)), 0.0

    h = cell_size / 2.0
    cx, cy = minx + width / 2.0, miny + height / 2.0
    best = _Cell(cx, cy, 0.0, point_to_polygon_distance(cx, cy, rings))

    queue, counter = [], 0
    x = minx
    while x < maxx:
        y = miny
        while y < maxy:
            cell = _Cell(x + h, y + h, h,
                         point_to_polygon_distance(x + h, y + h, rings))
            queue.append((-cell.max, counter, cell))
            counter += 1
            y += cell_size
        x += cell_size
    import heapq
    heapq.heapify(queue)

    while queue:
        neg_max, _, cell = heapq.heappop(queue)
        if cell.d > best.d:
            best = cell
        if (-neg_max) - best.d <= precision:
            continue
        hh = cell.h / 2.0
        for dx in (0.0, hh):
            for dy in (0.0, hh):
                sx, sy = cell.x - cell.h + dx, cell.y - cell.h + dy
                sub = _Cell(sx, sy, hh, point_to_polygon_distance(sx, sy, rings))
                heapq.heappush(queue, (-sub.max, counter, sub))
                counter += 1

    return (float(best.x), float(best.y)), max(0.0, float(best.d))


# --------------------------------------------------------------------------- #
# 2b. 占据栅格 + 距离变换：凹形 / 带天井屋顶的落点
# --------------------------------------------------------------------------- #
# 为什么需要这一套：`polylabel` 作用在**凸包**上，而凸包会把凹口和天井"填平"。
# 于是凹形屋顶（U 形、L 形、回字形天井）上求出的"最大内切圆圆心"可能根本不
# 在屋顶上——实测回字形屋顶 24×16 m 中央开 8×6 m 天井时，凸包法给出的落点
# 就在天井正中，无人机按这个点下降会直接掉进天井。
#
# 这里改用**占据栅格 + 欧氏距离变换**：先把点换成"哪些格子有屋顶"，再求每个
# 占据格到最近空格的距离，取最大者即真正的最大内切圆圆心。它天然尊重凹口、
# 天井与障碍，且依然只依赖 numpy。

def _shift_or(mask):
    """3×3 邻域"或"：带 padding 的平移，**不能用 np.roll**。

    坑：`np.roll` 会把最后一列环绕到第一列，于是栅格最外圈（本来是"没有点"的
    安全边界）被填成占据，距离变换算出来的就成了"到图外"的距离而不是到空隙的
    距离——实测 U 形屋顶因此报出 8.0 m 半径、落点跑到角落。
    """
    h, w = mask.shape
    padded = np.pad(mask, 1, mode="constant", constant_values=False)
    out = np.zeros_like(mask)
    for dy in range(3):
        for dx in range(3):
            out |= padded[dy:dy + h, dx:dx + w]
    return out


def _shift_and(mask):
    """3×3 邻域"与"（腐蚀），同样用 padding 避免环绕。"""
    h, w = mask.shape
    padded = np.pad(mask, 1, mode="constant", constant_values=False)
    out = np.ones_like(mask)
    for dy in range(3):
        for dx in range(3):
            out &= padded[dy:dy + h, dx:dx + w]
    return out


def _binary_dilate(mask):
    """3×3 膨胀（不引入 scipy）。"""
    return _shift_or(np.asarray(mask, dtype=bool))


def _binary_erode(mask):
    """3×3 腐蚀。"""
    return _shift_and(np.asarray(mask, dtype=bool))


def rasterize_occupancy(points_xy, cell=0.5, close_iters=1):
    """把平面内点栅格化成占据图。

    为什么默认做一次**闭运算**（先膨胀后腐蚀）：LiDAR 点云是稀疏采样，0.5 m 的
    格子里可能恰好没有点，形成"假空隙"；不填掉的话距离变换会把落点挤到点最密
    的地方而不是真正的几何中心。闭运算只填一格宽的缝，不会把天井（通常远宽于
    1 m）糊上。

    Args:
        points_xy: (N,2) 平面内点
        cell: 栅格边长 (m)
        close_iters: 闭运算次数（0 表示不做）

    Returns:
        dict（输入退化时返回 None）::

            mask    (H,W) bool    True = 该格有屋顶
            origin  (x0,y0)       mask[0,0] 格子的左下角坐标
            cell    float
            area    float         占据面积估计 (m^2)
    """
    pts = np.asarray(points_xy, dtype=np.float64)
    if pts.ndim != 2 or len(pts) < 3 or pts.shape[1] < 2:
        return None
    step = float(cell)
    if not math.isfinite(step) or step <= 0.0:
        return None

    xy = pts[:, :2]
    minx, miny = xy.min(axis=0) - step
    maxx, maxy = xy.max(axis=0) + step
    nx = int(math.ceil((maxx - minx) / step)) + 1
    ny = int(math.ceil((maxy - miny) / step)) + 1
    if nx < 2 or ny < 2:
        return None

    ix = np.floor((xy[:, 0] - minx) / step).astype(np.int64)
    iy = np.floor((xy[:, 1] - miny) / step).astype(np.int64)
    np.clip(ix, 0, nx - 1, out=ix)
    np.clip(iy, 0, ny - 1, out=iy)

    mask = np.zeros((ny, nx), dtype=bool)
    mask[iy, ix] = True
    for _ in range(int(close_iters)):
        mask = _binary_erode(_binary_dilate(mask))
    # 强制最外圈为空：距离变换需要"屋顶之外至少有一圈空地"才能算出真实余量；
    # 若点云恰好铺满栅格边界，这一步相当于把边界当作屋顶边缘处理（保守）。
    if mask.shape[0] >= 3 and mask.shape[1] >= 3:
        mask[0, :] = mask[-1, :] = False
        mask[:, 0] = mask[:, -1] = False
    if not mask.any():
        return None
    return {"mask": mask, "origin": (float(minx), float(miny)), "cell": step,
            "area": float(mask.sum()) * step * step}


def _edt_1d(f):
    """一维平方距离变换（Felzenszwalb & Huttenlocher 1985，精确算法）。

    ``f[i]`` 是第 i 个位置的初始代价（0 或 +inf），返回
    ``d[i] = min_j ((i-j)^2 + f[j])``。下面用它把 2D 距离变换拆成两趟 1D。
    """
    from collections import deque

    n = f.size
    d = np.empty(n, dtype=np.float64)
    # 直接用 O(n^2) 会让 200x200 的图慢到几秒；这里用论文里的下包络算法 O(n)
    v = np.zeros(n, dtype=np.int64)      # 抛物线顶点位置
    z = np.empty(n + 1, dtype=np.float64)  # 各段分界
    z[0] = -np.inf
    z[1] = np.inf
    k = 0
    for q in range(1, n):
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = np.inf
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def distance_transform(mask):
    """占据栅格的平方欧氏距离变换：每个占据格到最近空格的距离（单位：格^2）。

    ``mask`` 为 True 的格子是屋顶，False 是空隙。返回与 mask 同形的数组，
    空格处为 0。
    """
    m = np.asarray(mask, dtype=bool)
    if m.ndim != 2 or not m.any():
        return np.zeros_like(m, dtype=np.float64)
    big = 1e12
    f = np.where(m, big, 0.0)            # 空格代价 0，占据格代价 +inf
    # 先按列（y 方向），再按行（x 方向）
    tmp = np.empty_like(f)
    for j in range(f.shape[1]):
        tmp[:, j] = _edt_1d(f[:, j])
    out = np.empty_like(f)
    for i in range(f.shape[0]):
        out[i, :] = _edt_1d(tmp[i, :])
    return np.where(m, out, 0.0)


def point_in_occupancy(x, y, grid):
    """判断落点是否落在占据格里（即真的在屋顶上）。

    这是 ``auto`` 模式的**关键判据**：只看面积比会漏掉小天井——24×16 m 屋顶中央
    开个 8×6 m 的天井只占 12% 面积，面积比 0.88 看着"很正常"，但凸包落点正好
    落在天井中心。所以先直接问一句：凸包算出来的那个点，脚下有屋顶吗？
    """
    if not grid:
        return False
    mask, (x0, y0), step = grid["mask"], grid["origin"], grid["cell"]
    ix = int(math.floor((float(x) - x0) / step))
    iy = int(math.floor((float(y) - y0) / step))
    if 0 <= iy < mask.shape[0] and 0 <= ix < mask.shape[1]:
        return bool(mask[iy, ix])
    return False


def grid_incenter(points_xy, cell=0.5, close_iters=1):
    """占据栅格版「最大内切圆圆心」：在真实屋顶范围内找离空隙最远的格子。

    与 :func:`polylabel` 的区别：polylabel 吃的是**凸包多边形**，凹口/天井会被
    填平；这里吃的是**占据栅格**，凹口与天井自然是空格，因此落点不会落进去。

    返回 dict 或 None::

        landing_point (x,y)  落点（格子中心）
        radius        最大内切圆半径 (m)，换算时保守内缩一格（见下方注释）
        area          占据面积 (m^2)
        mask/origin/cell  便于可视化与复算
        shape_ratio   占据面积 / 凸包面积（凹形或带孔时明显小于 1）
    """
    grid = rasterize_occupancy(points_xy, cell=cell, close_iters=close_iters)
    if grid is None:
        return None
    mask, (x0, y0), step = grid["mask"], grid["origin"], grid["cell"]
    d2 = distance_transform(mask)
    if not np.isfinite(d2).any() or d2.max() <= 0.0:
        return None
    iy, ix = np.unravel_index(int(np.argmax(d2)), d2.shape)
    # 保守换算成米：距离变换算的是"到最近空格中心的格数"，而一个占据格代表
    # 边长 cell 的一小块、其真实边界最多再向外半格。为安全起见整体内缩一格
    # （宁可把余量少报 0.5 m，也不能多报——半径直接决定落点离女儿墙多远）。
    radius_cells = math.sqrt(float(d2[iy, ix])) - 1.0
    radius = max(0.0, radius_cells) * step
    return {"landing_point": (x0 + (ix + 0.5) * step, y0 + (iy + 0.5) * step),
            "radius": float(radius), "area": grid["area"], "mask": mask,
            "origin": (x0, y0), "cell": step}


def footprint_points(result, max_points=600):
    """把占据栅格结果转成一串 (x,y,z) 点，供 RViz 用 CUBE_LIST 画屋顶覆盖范围。

    凹形/带天井的屋顶用凸包线框看不出来（凸包把凹口填平了），所以除线框之外
    再画一层占据格，RViz 里能直接看出"哪里是屋顶、哪里是洞"。
    """
    grid = result.get("grid")
    if not grid:
        return []
    mask, (x0, y0), step = grid["mask"], grid["origin"], grid["cell"]
    iy, ix = np.nonzero(mask)
    if len(ix) > max_points:                     # 稀疏化，避免 marker 过大
        sel = np.linspace(0, len(ix) - 1, max_points).astype(np.int64)
        ix, iy = ix[sel], iy[sel]
    z = float(result.get("height", 0.0))
    return [(x0 + (i + 0.5) * step, y0 + (j + 0.5) * step, z)
            for i, j in zip(ix, iy)]


def split_clusters_2d(points, cell=1.0, min_points=2):
    """把（近似共面的）点集按水平面 2D 栅格做连通域聚类。

    平面内点里常混有零星离群点：远处的杂点、以及**邻近楼顶**的点，都可能落在
    同一个高度带里被 RANSAC 收进内点。凸包对离群点极端敏感 —— 少数几个远处的
    点就能把轮廓撑开，使「最大内切圆圆心」跑到真实屋顶之外。实测：6900 个点里
    混进 4 个离群点（占 0.06%），就把 216 m² 的屋顶撑成 1175 m²，降落点偏了 22 m。

    这里先把点集栅格化，丢掉点数不足 ``min_points`` 的零星格子，再对剩余格子做
    八邻域连通域标记，按点数从多到少返回各个连通块。这样既滤掉了零星噪点，
    也顺带把「相邻但不相连的两栋楼」拆成两个独立的候选屋顶。

    Args:
        points: (N,3) 或 (N,2) 点集，只用前两维做聚类
        cell: 栅格边长 (m)
        min_points: 一个格子至少要有几个点才算被占据

    Returns:
        list of ndarray，每个元素是一个连通块的点（保持原始列数）；
        输入为空或无法聚类时返回 ``[points]``。
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or len(pts) == 0 or pts.shape[1] < 2:
        return [pts]

    ij = np.floor(pts[:, :2] / float(cell)).astype(np.int64)
    uniq, inv, counts = np.unique(ij, axis=0, return_inverse=True,
                                  return_counts=True)
    inv = np.asarray(inv).ravel()
    kept = np.where(counts >= int(min_points))[0]
    if len(kept) == 0:
        return [pts]

    # 全量 uniq 下标 -> 保留格子下标 的映射，便于后面向量化取点
    k_of_full = -np.ones(len(uniq), dtype=np.int64)
    k_of_full[kept] = np.arange(len(kept))
    occupied = {tuple(c): k for k, c in enumerate(uniq[kept])}

    seen = set()
    comps = []
    for c0 in map(tuple, uniq[kept]):
        if c0 in seen:
            continue
        seen.add(c0)
        stack = [c0]
        members = []
        while stack:
            cur = stack.pop()
            members.append(occupied[cur])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (cur[0] + dx, cur[1] + dy)
                    if nb in occupied and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        comps.append(members)
    if not comps:
        return [pts]
    comps.sort(key=len, reverse=True)

    kk = k_of_full[inv]                 # 每个点对应的保留格子下标；-1 = 零星点
    valid = kk >= 0
    out = []
    for members in comps:
        flag = np.zeros(len(kept), dtype=bool)
        flag[np.asarray(members, dtype=np.int64)] = True
        sel = np.zeros(len(pts), dtype=bool)
        sel[valid] = flag[kk[valid]]
        if np.any(sel):
            out.append(pts[sel])
    return out if out else [pts]


# --------------------------------------------------------------------------- #
# 3. 屋顶检测主流程
# --------------------------------------------------------------------------- #
def detect_rooftops(points, max_tilt_deg=15.0, dist_thresh=0.15,
                    min_area=8.0, min_inliers=120, max_planes=5,
                    min_height=0.5, rng=None,
                    cluster_cell=1.0, cluster_min_points=2,
                    min_height_mode="relative", ground_margin=1.0, info=None,
                    landing_mode="auto", grid_cell=0.5, grid_close_iters=1,
                    concave_ratio=0.85):
    """从点云中找出所有「水平候选面」，按高度从高到低返回列表。

    每个元素::

        {
          "normal": (3,), "d": float, "height": float,
          "area": float, "hull": (M,2) ndarray,
          "landing_point": (x, y), "radius": float,
          "landing_mode": "hull" | "grid",   # 落点由哪种方法给出
          "shape_ratio": float,              # 占据面积 / 凸包面积
          "grid": dict | None,               # grid 模式下的占据栅格，供可视化
        }

    求凸包之前会先用 :func:`split_clusters_2d` 把平面上互不相连的点群拆开
    （``cluster_cell`` / ``cluster_min_points``），因此**一个平面可能产生多个
    候选屋顶** —— 既滤掉了零星离群点，也把相邻的两栋楼分成两个独立候选。

    **落点算法 ``landing_mode``**（为什么默认不是纯凸包，见 §2b 的注释）：

    ``"auto"``（默认）
        先用凸包 + polylabel 求一版落点，同时用占据栅格 + 距离变换求一版；
        若 ``占据面积 / 凸包面积 < concave_ratio``（说明轮廓明显不凸或带天井），
        采用栅格版落点，否则保持凸包版。**凸形屋顶的行为与旧版本完全一致**，
        只有凹口/天井这类"凸包会骗人"的形状才切换。
    ``"grid"``
        强制用占据栅格版（凹形/带孔屋顶更安全，代价是需要足够密的点云）。
    ``"hull"``
        强制用旧的凸包 + polylabel（回归对照用；凹形屋顶上可能把落点放在天井里）。

    **高度下限 ``min_height_mode``**（地面与屋顶都是水平面，只能靠高度区分）：

    ``"relative"``（默认）
        以**点云里检出的最低水平面（地面）**为基准，丢弃与它高度差小于
        ``ground_margin`` 的平面。换地图、原点不在作业现场也能直接用。
        实测教训：某地图的原点在地面之上约 25 m，所有平面高度都是 −25 左右，
        固定阈值 ``min_height=0.5`` 会把**每一个**候选面都滤掉，日志表现为
        「检测到 0 个候选水平面」，无人机一直不动；而把阈值放到 −1000 之后
        **地面又被当成屋顶**，会在地面上"降落"。
        只有提取到 **2 个及以上**水平面时才把最低的那个当基准 —— 只有一个
        水平面时无从判断它是地面还是屋顶，此时不做高度过滤。
    ``"absolute"``
        直接用 ``min_height`` 作为绝对 ENU 高度下限（旧行为），
        适合原点就在地面高度上的地图。

    传入 ``info`` 字典（可选）可取出本次判定依据::

        info["planes"]     提取到的水平面个数
        info["floor_ref"]  地面参考高度（relative 模式下有效，否则 None）
        info["threshold"]  实际生效的高度下限
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return []

    # ---- 第一阶段：把水平面全部提出来（先不做高度过滤，好判断地面在哪）----
    remaining = pts.copy()
    planes = []
    for _ in range(int(max_planes)):
        if len(remaining) < min_inliers:
            break
        plane = fit_plane_ransac(remaining, dist_thresh=dist_thresh,
                                 min_inliers=min_inliers, rng=rng)
        if plane is None:
            break
        plane_pts = remaining[plane["inliers"]]
        remaining = remaining[~plane["inliers"]]
        if plane_tilt_deg(plane["normal"]) > max_tilt_deg:
            continue                      # 非水平面（墙、坡面、树冠）
        height = plane_height(plane["normal"], plane["d"])
        if not math.isfinite(height):
            continue
        planes.append({"normal": plane["normal"], "d": plane["d"],
                       "height": float(height), "points": plane_pts})

    # ---- 高度下限：相对地面 or 绝对 ----
    floor_ref = None
    if min_height_mode == "relative":
        if len(planes) >= 2:
            floor_ref = min(p["height"] for p in planes)
            threshold = floor_ref + abs(ground_margin)
        else:
            # 只有一个水平面，无从判断它是地面还是屋顶 ⇒ 不做过高度滤
            threshold = float("-inf")
    else:
        threshold = min_height
    if info is not None:
        info["planes"] = len(planes)
        info["floor_ref"] = floor_ref
        info["threshold"] = float(threshold)

    # ---- 第二阶段：过滤 + 连通域拆分 + 凸包 + 最大内切圆 ----
    results = []
    for plane in planes:
        if plane["height"] < threshold:
            continue
        # 鲁棒性：先把平面上互不相连的点群拆开，各自求凸包。
        # 否则少数远处的高程噪点（或邻近楼顶）会把凸包撑开，
        # 使「最大内切圆圆心」落到真实屋顶之外。
        for cluster in split_clusters_2d(plane["points"], cell=cluster_cell,
                                         min_points=cluster_min_points):
            if len(cluster) < min_inliers:
                continue
            hull = convex_hull_2d(cluster)
            area = polygon_area(hull)
            if len(hull) < 3 or area < min_area:
                continue
            (lx, ly), radius = polylabel([hull], precision=0.05)
            mode, shape_ratio, grid = "hull", 1.0, None
            if landing_mode in ("auto", "grid"):
                g = grid_incenter(cluster[:, :2], cell=grid_cell,
                                  close_iters=grid_close_iters)
                if g is not None and area > 0.0:
                    shape_ratio = float(g["area"] / area)
                    grid = {k: g[k] for k in ("mask", "origin", "cell", "area")}
                    # auto 的切换条件（两条任一满足就换栅格落点）：
                    #   ① 轮廓明显不凸/带孔：占据面积显著小于凸包面积；
                    #   ② 凸包算出来的落点脚下根本没有屋顶（落进天井/凹口）——
                    #      面积比可能看起来很正常，但这条是安全底线。
                    hull_pt_on_roof = point_in_occupancy(lx, ly, grid)
                    if (landing_mode == "grid"
                            or shape_ratio < float(concave_ratio)
                            or not hull_pt_on_roof):
                        lx, ly = g["landing_point"]
                        radius = g["radius"]
                        mode = "grid"
                    else:
                        grid = None
            results.append({"normal": plane["normal"], "d": plane["d"],
                            "height": plane["height"], "area": float(area),
                            "hull": hull, "landing_point": (float(lx), float(ly)),
                            "radius": float(radius), "landing_mode": mode,
                            "shape_ratio": float(shape_ratio), "grid": grid})

    results.sort(key=lambda r: r["height"], reverse=True)
    return results


def select_landing_target(rooftops, current_xy=None, prefer="highest",
                          min_radius=0.6, max_distance=None):
    """从候选屋顶里选一个降落目标，返回元素或 None。

    ``min_radius`` 是**硬性安全约束**：内切圆半径小于它的候选一律不用；
    一个候选都不满足时返回 ``None``，由调用方继续搜索。早期版本在这种情况下
    会**回退到全部候选**，等于让无人机落在一个四周余量不足的屋顶上，
    与「安全半径」的初衷相反，已去掉。

    ``prefer``:
        ``"highest"``   —— 选最高的（最像「屋顶」而不是「地面」）
        ``"closest"``   —— 选离当前位置水平距离最近的
        ``"largest"``   —— 选内切圆半径最大的（最安全的）

    ``max_distance`` 是**软约束**：只在存在满足者时用来缩小范围，都不满足时
    仍在全部候选里挑，避免因为屋顶都偏远就彻底放弃降落。
    """
    cands = [r for r in rooftops if r["radius"] >= min_radius]
    if not cands:
        return None

    def dist(r):
        if current_xy is None:
            return 0.0
        return math.hypot(r["landing_point"][0] - current_xy[0],
                          r["landing_point"][1] - current_xy[1])

    if max_distance is not None:
        limited = [r for r in cands if dist(r) <= max_distance]
        if limited:
            cands = limited

    if prefer == "closest" and current_xy is not None:
        return min(cands, key=dist)
    if prefer == "largest":
        return max(cands, key=lambda r: r["radius"])
    return max(cands, key=lambda r: (r["height"], r["radius"]))
