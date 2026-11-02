"""场景世界：轴对齐墙体 + 圆柱路标 + 一个"来回移动"的动态障碍。

几何全部是二维的（俯视），用解析求交实现单目相机的光线投射，
因此不需要任何渲染引擎，纯 NumPy 可在任意机器上复现。
"""

import math

import numpy as np

from . import params as P

# 12 个路标颜色，两两在 RGB 空间距离足够大，便于按颜色做数据关联
PALETTE = np.array([
    [0.95, 0.20, 0.15],   # 0 红
    [0.15, 0.80, 0.25],   # 1 绿
    [0.20, 0.35, 0.95],   # 2 蓝
    [0.98, 0.85, 0.10],   # 3 黄
    [0.85, 0.25, 0.85],   # 4 品红
    [0.10, 0.85, 0.85],   # 5 青
    [0.95, 0.55, 0.15],   # 6 橙
    [0.55, 0.30, 0.90],   # 7 紫
    [0.60, 0.90, 0.35],   # 8 黄绿
    [0.90, 0.55, 0.70],   # 9 粉
    [0.45, 0.65, 0.85],   # 10 灰蓝
    [0.75, 0.75, 0.30],   # 11 橄榄
], dtype=np.float64)


def _rect(x0, y0, x1, y1):
    return [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]


class World:
    """静态障碍（AABB）+ 圆柱路标 + 可选动态障碍。"""

    def __init__(self, rects, landmarks, start, goal, dynamic=None):
        self.rects = np.asarray(rects, dtype=np.float64).reshape(-1, 4)
        self.landmarks = np.asarray(landmarks, dtype=np.float64).reshape(-1, 3)
        self.start = np.asarray(start, dtype=np.float64)
        self.goal = np.asarray(goal, dtype=np.float64)
        # dynamic: (cx, cy, r, amp, omega, phase, axis)，axis=0 沿 x、1 沿 y
        self.dynamic = None if dynamic is None else np.asarray(dynamic, dtype=np.float64)
        self.t = 0.0

    # ------------------------------------------------------------ 动态障碍
    def dynamic_center(self):
        if self.dynamic is None:
            return None
        cx, cy, _r, amp, omega, phase, axis = self.dynamic
        off = amp * math.sin(omega * self.t + phase)
        return np.array([cx + off, cy]) if int(axis) == 0 else np.array([cx, cy + off])

    def circles(self):
        """返回当前时刻所有圆形障碍 (M,3)，前 M_landmark 个是路标。"""
        lm = self.landmarks[:, :2]
        r = np.full((len(lm), 1), P.LM_RADIUS)
        out = np.hstack([lm, r])
        c = self.dynamic_center()
        if c is not None:
            out = np.vstack([out, [c[0], c[1], self.dynamic[2]]])
        return out

    def step(self, dt):
        self.t += dt

    # ------------------------------------------------------------ 求交
    def raycast(self, origin, angles, max_range=None):
        """对一批射线求最近命中。

        返回 (dist, hid)：hid=0 表示未命中；1..N 为墙体索引+1；
        N+1..N+M 为路标索引+1；最后一个是动态障碍（若有）。
        """
        max_range = P.CAM_RANGE_MAX if max_range is None else max_range
        ox, oy = float(origin[0]), float(origin[1])
        ang = np.asarray(angles, dtype=np.float64).reshape(-1)
        dx, dy = np.cos(ang), np.sin(ang)
        K = ang.size

        best = np.full(K, max_range, dtype=np.float64)
        hid = np.zeros(K, dtype=np.int32)

        # --- 轴对齐矩形（slab 法）
        if len(self.rects):
            nx, fx = _slab(ox, dx, self.rects[:, 0], self.rects[:, 2])
            ny, fy = _slab(oy, dy, self.rects[:, 1], self.rects[:, 3])
            t_in = np.maximum(nx, ny)
            t_out = np.minimum(fx, fy)
            hit = (t_in <= t_out) & (t_out > 1e-6)
            t = np.where(t_in > 1e-6, t_in, t_out)
            t = np.where(hit, t, np.inf)
            idx = np.argmin(t, axis=1)
            tb = t[np.arange(K), idx]
            m = tb < best
            best[m] = tb[m]
            hid[m] = (idx[m] + 1).astype(np.int32)

        # --- 圆（二次方程取近根）
        cir = self.circles()
        if len(cir):
            cx = cir[:, 0][None, :] - ox
            cy = cir[:, 1][None, :] - oy
            b = cx * dx[:, None] + cy * dy[:, None]
            c0 = cx * cx + cy * cy - (cir[:, 2] ** 2)[None, :]
            disc = b * b - c0
            ok = disc > 0
            sq = np.sqrt(np.where(ok, disc, 0.0))
            # cx = c - o，故 m·d = -b，近根为 b - sqrt(b^2 - c0)
            t = b - sq
            t = np.where(ok & (t > 1e-6), t, np.inf)
            idx = np.argmin(t, axis=1)
            tb = t[np.arange(K), idx]
            m = tb < best
            best[m] = tb[m]
            hid[m] = (idx[m] + 1 + len(self.rects)).astype(np.int32)

        return best, hid

    # ------------------------------------------------------------ 占用查询
    def point_free(self, x, y, margin=0.0):
        """点是否在自由空间（含车体外扩 margin）。"""
        if not (-margin < x < P.WORLD_W + margin and -margin < y < P.WORLD_H + margin):
            return False
        for x0, y0, x1, y1 in self.rects:
            if (x0 - margin) <= x <= (x1 + margin) and (y0 - margin) <= y <= (y1 + margin):
                return False
        for cx, cy, r in self.circles():
            if (x - cx) ** 2 + (y - cy) ** 2 <= (r + margin) ** 2:
                return False
        return True


def _slab(o, d, lo, hi):
    """一条射线在一个轴向上与区间 [lo,hi] 的进出参数。"""
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (lo[None, :] - o) / d[:, None]
        t2 = (hi[None, :] - o) / d[:, None]
    near = np.minimum(t1, t2)
    far = np.maximum(t1, t2)
    para = np.abs(d) < 1e-9
    if para.any():
        inside = (o >= lo) & (o <= hi)
        near = np.where(para[:, None], np.where(inside, -np.inf, np.inf)[None, :], near)
        far = np.where(para[:, None], np.where(inside, np.inf, -np.inf)[None, :], far)
    return near, far


def _base_layout():
    """三房间 S 形走廊：门洞交错，必须真规划才能通过。"""
    t = P.WALL_THICK
    rects = [
        _rect(0, 0, P.WORLD_W, t),                    # 下
        _rect(0, P.WORLD_H - t, P.WORLD_W, P.WORLD_H),  # 上
        _rect(0, 0, t, P.WORLD_H),                    # 左
        _rect(P.WORLD_W - t, 0, P.WORLD_W, P.WORLD_H),  # 右
        _rect(6.85, 0, 7.0, 9.4),                     # 隔墙 A，门洞在上方 y>9.4
        _rect(12.85, 4.6, 13.0, P.WORLD_H),           # 隔墙 B，门洞在下方 y<4.6
    ]
    return rects


def _landmark_layout():
    """12 个路标，沿可行路径两侧分布，保证起步后很快能观测到若干个。"""
    return [
        (3.6, 4.6, 0), (5.4, 3.2, 1), (8.8, 1.8, 2),
        (9.0, 8.0, 3), (5.3, 11.8, 4), (10.0, 12.2, 5),
        (11.4, 7.2, 6), (14.6, 5.4, 7), (15.8, 8.6, 8),
        (14.6, 11.4, 9), (17.8, 9.8, 10), (18.4, 4.2, 11),
    ]


def make_scene(key, seed=0):
    """构造四个场景之一。场景差异只体现在障碍/光照/动态上，起终点一致。

    cluttered 的随机堆箱可能把通道堵死，因此构造后必须用真值栅格做一次
    可达性检查：不可达就换一组随机数重来。否则"成功率"里会混进一批
    物理上无解的场面，评测没有意义。
    """
    if key in ('corridor', 'cluttered'):
        for attempt in range(40):
            w = _build(key, seed * 1000 + attempt)
            if _reachable(w):
                return w
        return _build(key, seed * 1000)
    return _build(key, seed)


def _build(key, seed):
    rng = np.random.default_rng(seed)
    rects = _base_layout()
    lms = [list(p) for p in _landmark_layout()]
    dynamic = None

    if key == 'corridor':
        for i in range(len(lms)):
            lms[i][0] += rng.normal(0.0, 0.10)
            lms[i][1] += rng.normal(0.0, 0.10)

    elif key == 'cluttered':
        # 房间内随机堆 10 个小箱体，避开路标、门洞与起终点
        placed = 0
        tries = 0
        while placed < 6 and tries < 400:
            tries += 1
            cx = rng.uniform(1.2, P.WORLD_W - 1.2)
            cy = rng.uniform(1.2, P.WORLD_H - 1.2)
            s = rng.uniform(0.24, 0.36)
            cand = _rect(cx - s, cy - s, cx + s, cy + s)
            if abs(cx - 7.0) < 1.1 or abs(cx - 13.0) < 1.1:
                continue
            if math.hypot(cx - 2.0, cy - 2.0) < 2.2:
                continue
            if math.hypot(cx - 17.6, cy - 11.6) < 2.2:
                continue
            if not _clear_of(cand, lms, gap=0.75):
                continue
            rects.append(cand)
            placed += 1

    elif key == 'dynamic':
        # 第二房间里一个 0.30 m 的箱子沿 x 轴往复，横穿走廊，考验在线避障。
        # 巡逻带 x∈[8.2, 9.8]：与路标 #6 (11.4, 7.2) 保持 1.3 m 以上间距，
        # 否则车被箱子逼停后做恢复动作时会飘进路标的碰撞半径。
        dynamic = (9.0, 7.0, 0.30, 0.8, 0.50, 0.0, 0)

    elif key == 'night':
        pass
    else:
        raise ValueError('unknown scene: %s' % key)

    return World(rects, lms, start=(2.0, 2.0, 0.0), goal=(17.6, 11.6),
                 dynamic=dynamic)


def _reachable(w):
    """真值栅格上起点到终点是否存在一条给车留够余量的通路。"""
    from .gridmap import truth_grid
    from .planner import astar, inflate

    if not w.point_free(w.start[0], w.start[1], margin=0.35):
        return False
    blocked = inflate(truth_grid(w), 4)
    path = astar(blocked, (w.start[0], w.start[1]), (w.goal[0], w.goal[1]))
    return len(path) > 0


def _clear_of(cand, lms, gap):
    for x, y, _c in lms:
        if cand[0] - gap <= x <= cand[2] + gap and cand[1] - gap <= y <= cand[3] + gap:
            return False
    return True
