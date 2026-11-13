"""牛耕式全覆盖规划（boustrophedon / lawnmower coverage）。

前两类导航（势场、Bug2）回答"怎么到某个点"；全覆盖回答的是另一个问题：
**怎么把整个房间都走一遍**——扫地机器人、农田植保、光伏巡检都是这一类。

牛耕式（boustrophedon，希腊语"牛耕地"）是最简单的全覆盖方案：

1. 选一个扫掠方向，用一组等距平行线切多边形房间；
2. 逐行扫掠，行末掉头换行（相邻行方向相反，像"之"字）；
3. 掉头连接若直接穿出房间（凹形边界），改为**沿边界走短弧**。

它的三个基本性质：
- **完备覆盖**：行距 gap ≤ 作业宽度时，扫过区域无空洞；
- **路径长度可预算**：length ≈ 面积 / gap + 掉头开销，适合估算作业时间；
- **无全局搜索**：没有 A*/RRT 的图搜索，纯几何求交。

障碍或凹形（房间里的内墙、立柱、U 形障碍）通过"行区间被切成多段"
自动处理：每行只在多边形内部的区间上扫。纯函数实现，便于离线单测。
"""

import math

# 演示房间：20 x 20 的 L 形（顶点逆时针）
ROOM_RECT = [(5.0, 5.0), (35.0, 5.0), (35.0, 15.0),
             (20.0, 15.0), (20.0, 35.0), (5.0, 35.0)]
ROOM_L = ROOM_RECT

DEFAULT_START = (5.0, 5.0)


# ----------------------------------------------------------------------------
# 多边形基础
# ----------------------------------------------------------------------------
def polygon_area(poly):
    """有符号面积（鞋带公式），逆时针为正。"""
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return a / 2.0


def point_in_polygon(q, poly):
    """点是否在多边形内（射线法，边界算在内部）。"""
    x, y = q
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        # 点恰在边上 → 内部
        if _on_segment(q, (x1, y1), (x2, y2)):
            return True
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xin:
                inside = not inside
    return inside


def _on_segment(q, a, b):
    if dist_to_segment(q, a, b) < 1e-9:
        return True
    return False


def dist_to_segment(q, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    d2 = dx * dx + dy * dy
    if d2 < 1e-18:
        return math.hypot(q[0] - ax, q[1] - ay)
    t = max(0.0, min(1.0, ((q[0] - ax) * dx + (q[1] - ay) * dy) / d2))
    return math.hypot(q[0] - (ax + t * dx), q[1] - (ay + t * dy))


def segment_crosses_boundary(a, b, poly):
    """线段 a-b 是否与多边形边界相交（含穿过；端点在边界上不算）。"""
    n = len(poly)
    for i in range(n):
        p1, p2 = poly[i], poly[(i + 1) % n]
        d1 = _cross(p1, p2, a)
        d2 = _cross(p1, p2, b)
        d3 = _cross(a, b, p1)
        d4 = _cross(a, b, p2)
        if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
            return True
    return False


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


# ----------------------------------------------------------------------------
# 扫掠线与多边形求交
# ----------------------------------------------------------------------------
def sweep_row(y, poly, theta=0.0):
    """一条扫掠线与多边形的交区间，返回 [(xa, xb), ...]（xa < xb）。

    坐标系先旋转 -theta 到"水平扫掠"坐标，再转回。
    """
    # 旋转到局部坐标（把扫掠方向转到 x 轴）
    c, s = math.cos(-theta), math.sin(-theta)
    lpoly = [(_rot(p, c, s)) for p in poly]
    ylo = min(p[1] for p in lpoly)
    yhi = max(p[1] for p in lpoly)
    if y < ylo - 1e-12 or y > yhi + 1e-12:
        return []

    xs = []
    n = len(lpoly)
    for i in range(n):
        (x1, y1), (x2, y2) = lpoly[i], lpoly[(i + 1) % n]
        if abs(y1 - y2) < 1e-15:
            continue                      # 平行边不产生交点
        lo, hi = (y1, y2) if y1 < y2 else (y2, y1)
        if lo <= y <= hi:
            t = (y - y1) / (y2 - y1)
            xs.append(x1 + t * (x2 - x1))
    xs.sort()
    spans = []
    for i in range(0, len(xs) - 1, 2):
        if xs[i + 1] - xs[i] > 1e-9:
            spans.append((xs[i], xs[i + 1]))
    # 转回全局坐标
    c2, s2 = math.cos(theta), math.sin(theta)
    return [(_rot((xa, y), c2, s2), _rot((xb, y), c2, s2)) for xa, xb in spans]


def _rot(p, c, s):
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


# ----------------------------------------------------------------------------
# 主规划
# ----------------------------------------------------------------------------
def plan(poly, gap=2.0, theta=0.0):
    """生成牛耕式全覆盖路径。

    返回 dict：
        path      [(x, y), ...]  连续路径（含掉头连接）
        rows      扫掠行数
        length    路径总长
        est_time  1 m/s 匀速的预估作业时间
        （采样覆盖率请另调 coverage_stats 精算）
    """
    if polygon_area(poly) < 0:
        poly = list(reversed(poly))       # 统一逆时针

    # 行中心从"半行距 + 微偏移"开始：半行距保证边界的作业宽度也覆盖到；
    # 微偏移 epsilon 专治"行中心恰好压在水平边上"的退化（评审 #1400：
    # L 形房间取 theta=90° 时行中心正好落在 x=20 内墙上，sweep_row 跳过
    # 平行边后该行只扫了半段，留下 3% 的漏扫带，还伪装成"最短路径"）。
    c, s = math.cos(-theta), math.sin(-theta)
    ys = sorted(_rot(p, c, s)[1] for p in poly)
    y0, y1 = ys[0], ys[-1]
    eps = gap * 1e-3

    rows = []
    used_y = []
    y = y0 + gap / 2.0 + eps
    while y < y1 - 1e-9:
        for span in sweep_row(y, poly, theta):
            rows.append((span[0], span[1]))
        used_y.append(y)
        y += gap
    # 最后一行之后若还剩大半个行距未覆盖，补一行贴边扫掠
    if used_y and y1 - used_y[-1] > gap * 0.75 + 1e-9:
        for span in sweep_row(y1 - gap / 2.0 + eps, poly, theta):
            rows.append((span[0], span[1]))
        used_y.append(y1 - gap / 2.0 + eps)

    if not rows:
        # 房间比行距还窄：没有可扫的行，返回空路径而不是崩溃
        # （评审 #1400：此前这里多传了一个 0.0，参数错位后 polygon_area
        # 拿到 float 直接 TypeError）
        return _result([], 0, poly, gap, theta)

    # 组装路径：行内正向/反向交替，行间连接
    path = []
    rev = False
    prev_end = None
    for (a, b) in rows:
        seg = [a, b] if not rev else [b, a]
        if prev_end is not None:
            conn = _connect(prev_end, seg[0], poly)
            path.extend(conn[1:] if conn else [seg[0]])
        else:
            path.append(seg[0])
        path.append(seg[1])
        prev_end = seg[1]
        rev = not rev

    return _result(path, len(rows), poly, gap, theta)


def _connect(a, b, poly):
    """行间掉头：能直连且不穿出边界就用直线，否则沿边界走短弧。"""
    if not segment_crosses_boundary(a, b, poly):
        mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        if point_in_polygon(mid, poly):
            return [a, b]
    # 沿多边形边界走：找 a、b 在边界上的最近点，取两方向中的短者
    ia, sa = _boundary_param(a, poly)
    ib, sb = _boundary_param(b, poly)
    per = _perimeter(poly)
    fwd = (sb - sa) % per
    bwd = (sa - sb) % per
    pts = [a]
    if fwd <= bwd:
        n = max(2, int(fwd / 0.5) + 1)
        for k in range(1, n):
            pts.append(_point_at((sa + fwd * k / n) % per, poly))
    else:
        n = max(2, int(bwd / 0.5) + 1)
        for k in range(1, n):
            pts.append(_point_at((sa - bwd * k / n) % per, poly))
    pts.append(b)
    return pts


def _perimeter(poly):
    n = len(poly)
    return sum(math.hypot(poly[(i + 1) % n][0] - poly[i][0],
                          poly[(i + 1) % n][1] - poly[i][1])
               for i in range(n))


def _boundary_param(q, poly):
    """点 q 到边界最近点的 (边索引 i, 绝对弧长 s)。s 从边 0 的起点累计。"""
    best = (float('inf'), 0, 0.0)
    n = len(poly)
    acc = 0.0
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(dx, dy)
        d = dist_to_segment(q, a, b)
        if d < best[0]:
            t = 0.0 if seg < 1e-18 else max(0.0, min(1.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / (seg * seg)))
            best = (d, i, acc + t * seg)
        acc += seg
    return best[1], best[2]


def _point_at(s, poly):
    """绝对弧长 s 处的边界点（从边 0 的起点起算）。"""
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if s <= seg + 1e-12:
            t = 0.0 if seg < 1e-12 else s / seg
            return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        s -= seg
    return poly[0]


def _result(path, rows, poly, gap, theta, length=None):
    if length is None:
        length = sum(math.hypot(path[i + 1][0] - path[i][0],
                                path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))
    return {
        "path": path,
        "rows": rows,
        "length": length,
        "area": abs(polygon_area(poly)),
        "est_time": length / 1.0,          # 1 m/s 匀速
        "gap": gap,
        "theta": theta,
    }


# ----------------------------------------------------------------------------
# 覆盖率统计
# ----------------------------------------------------------------------------
def coverage_stats(poly, path, w=2.2, step=0.5):
    """在多边形内均匀采样，统计到路径距离 <= w/2 的点比例。

    w 是**作业宽度**（扫过一条行带的宽度），与行距 gap 解耦、不随 gap
    缩放——这正是覆盖率指标的意义所在（评审 #1400：旧版判定半径
    gap*0.55 随 gap 一起放大，行距拉到多大都会"自证"100%，看不出
    漏扫）。默认 2.2 m = 演示默认行距 2.0 m + 10% 重叠，与实际清扫 /
    植保作业一致；gap ≤ w 时满覆盖，gap > w 后行间出现宽 gap-w 的
    漏扫带，覆盖率约 w/gap。
    """
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    r = w / 2.0
    reached = total = 0
    y = y0
    while y <= y1:
        x = x0
        while x <= x1:
            q = (x, y)
            if point_in_polygon(q, poly):
                total += 1
                d = min(dist_to_segment(q, path[i], path[i + 1])
                        for i in range(len(path) - 1))
                if d <= r:
                    reached += 1
            x += step
        y += step
    return (reached / total) if total else 0.0
