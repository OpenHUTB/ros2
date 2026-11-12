"""禁飞区膨胀 + 可见图最短绕行的纯算法实现（无 ROS 依赖）。"""

import heapq
import math

ZONES = [
    [(5.0, -1.2), (7.0, -1.2), (7.0, 1.0), (5.0, 1.0)],
    [(10.0, -1.0), (12.0, -1.0), (12.0, 1.2), (10.0, 1.2)],
]
START = (0.0, 0.0)
GOAL = (15.0, 0.0)


def signed_area(poly):
    s = 0.0
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def _to_ccw(poly):
    return poly if signed_area(poly) > 0 else list(reversed(poly))


def _edges(poly):
    return list(zip(poly, poly[1:] + poly[:1]))


def _line_intersection(p1, d1, p2, d2):
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-12:
        return None
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / denom
    return p1[0] + t * d1[0], p1[1] + t * d1[1]


def inflate_polygon(poly, margin):
    """凸多边形向外膨胀 margin。"""
    pts = _to_ccw(list(poly))
    n = len(pts)
    lines = []
    for i in range(n):
        p1, p2 = pts[i], pts[(i + 1) % n]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        nx, ny = dy / length, -dx / length
        lines.append(((p1[0] + nx * margin, p1[1] + ny * margin), (dx, dy)))
    out = []
    m = len(lines)
    for i in range(m):
        p1, d1 = lines[i - 1]
        p2, d2 = lines[i]
        hit = _line_intersection(p1, d1, p2, d2)
        if hit is not None:
            out.append(hit)
    return out


def point_in_polygon(point, poly):
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in _edges(poly):
        if (y1 > y) != (y2 > y):
            xin = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < xin:
                inside = not inside
    return inside


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a, b, c, d, eps=1e-9):
    d1 = _orient(a, b, c)
    d2 = _orient(a, b, d)
    d3 = _orient(c, d, a)
    d4 = _orient(c, d, b)
    if ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and \
       ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps)):
        return True

    def on_seg(p, q, r):
        return (min(p[0], q[0]) - eps <= r[0] <= max(p[0], q[0]) + eps and
                min(p[1], q[1]) - eps <= r[1] <= max(p[1], q[1]) + eps)

    if abs(d1) <= eps and on_seg(a, b, c):
        return True
    if abs(d2) <= eps and on_seg(a, b, d):
        return True
    if abs(d3) <= eps and on_seg(c, d, a):
        return True
    if abs(d4) <= eps and on_seg(c, d, b):
        return True
    return False


def _proper_cross(a, b, c, d, eps=1e-9):
    d1 = _orient(a, b, c)
    d2 = _orient(a, b, d)
    d3 = _orient(c, d, a)
    d4 = _orient(c, d, b)
    return ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and \
           ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps))


def _close(p, q, eps=1e-9):
    return abs(p[0] - q[0]) <= eps and abs(p[1] - q[1]) <= eps


def segment_clear(a, b, poly):
    """线段 ab 是否不穿入多边形内部（允许与顶点/边相切）。"""
    mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    if point_in_polygon(mid, poly):
        return False
    for p, q in _edges(poly):
        if _proper_cross(a, b, p, q):
            return False
        if segments_intersect(a, b, p, q):
            if _close(a, p) or _close(a, q) or _close(b, p) or _close(b, q):
                continue
            return False
    return True


def path_blocked(start, goal, poly):
    return not segment_clear(start, goal, poly)


def clearance(point, poly):
    best = float("inf")
    px, py = point
    for (x1, y1), (x2, y2) in _edges(poly):
        dx, dy = x2 - x1, y2 - y1
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 < 1e-12 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg2))
        cx, cy = x1 + t * dx, y1 + t * dy
        best = min(best, math.hypot(px - cx, py - cy))
    return -best if point_in_polygon(point, poly) else best


def nearest_boundary_point(point, poly):
    """点到多边形边界的最近点。"""
    best = None
    for (x1, y1), (x2, y2) in _edges(poly):
        dx, dy = x2 - x1, y2 - y1
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 < 1e-12 else max(0.0, min(1.0, ((point[0] - x1) * dx + (point[1] - y1) * dy) / seg2))
        cand = (x1 + t * dx, y1 + t * dy)
        d = math.hypot(cand[0] - point[0], cand[1] - point[1])
        if best is None or d < best[0]:
            best = (d, cand)
    return best[1] if best else point


def visible(a, b, inflated):
    return all(segment_clear(a, b, poly) for poly in inflated)


def plan_path(start, goal, zones, margin=0.6):
    """在膨胀多边形的顶点上建可见图，用 Dijkstra 求最短绕行路径。

    返回从 start 到 goal 的点列；无解时返回 [start, goal] 并带上 blocked 标记。
    """
    inflated = [inflate_polygon(z, margin) for z in zones]
    for poly in inflated:
        if point_in_polygon(start, poly):
            start = nearest_boundary_point(start, poly)
            break
    if visible(start, goal, inflated):
        return [start, goal], False

    nodes = [start]
    for poly in inflated:
        nodes.extend(poly)
    nodes.append(goal)
    n = len(nodes)

    dist = [float("inf")] * n
    prev = [-1] * n
    dist[0] = 0.0
    heap = [(0.0, 0)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        for v in range(1, n):
            if not visible(nodes[u], nodes[v], inflated):
                continue
            nd = d + math.hypot(nodes[u][0] - nodes[v][0], nodes[u][1] - nodes[v][1])
            if nd < dist[v] - 1e-12:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    if dist[n - 1] == float("inf"):
        return [start, goal], True

    path = []
    cur = n - 1
    while cur != -1:
        path.append(nodes[cur])
        cur = prev[cur]
    path.reverse()
    return path, False


def next_waypoint(path, pos, accept_radius=0.5):
    """返回路径上第一个还没到达的点，以及它在路径中的下标。

    先从最近点出发，避免把已经走过的点当成目标而往回开。
    """
    nearest = min(range(len(path)),
                  key=lambda k: math.hypot(path[k][0] - pos[0], path[k][1] - pos[1]))
    for i in range(nearest, len(path)):
        p = path[i]
        if math.hypot(p[0] - pos[0], p[1] - pos[1]) > accept_radius:
            return p, i
    return path[-1], len(path) - 1
