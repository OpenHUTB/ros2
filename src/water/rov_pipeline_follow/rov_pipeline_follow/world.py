"""海底世界：海底管道 + 声应答器 + 海流。

几何全部是二维的（俯视海床平面）。管道用 Catmull-Rom 样条生成稠密
折线，相机光线对管道做"胶囊"求交（线段 + 半径），对特征做圆求交，
因此不需要任何渲染引擎，纯 NumPy 可在任意机器上复现。

任务定义（往返巡检）：ROV 从管道首端 A 出发，沿管道巡检到末端 B，
再沿管道返回 A —— 回到起点时 EKF-SLAM 的位姿误差就是回环闭合误差。
"""

import math

import numpy as np

from . import params as P

# 应答器颜色编码：按归一化色度空间挑选，保证
#   1) 两两间距 >= 0.19（分类阈值 0.10，留足噪声余量）；
#   2) 与水色（蓝绿）距离 >= 0.20 —— 早期版本的灰蓝/青两个颜色与水
#      背景的色度只差 0.09，整幅水背景会被误分类成应答器，真正的
#      应答器反而混在大块背景里检测不出来；
#   3) 与管道橙（工程橙，色度 [0.65, 0.32, 0.03]）距离 >= 0.19，
#      避免管道被当成标记、标记被当成管道。
PALETTE = np.array([
    [0.85, 0.15, 0.10],   # 0 红
    [0.92, 0.88, 0.15],   # 1 黄
    [0.55, 0.95, 0.25],   # 2 黄绿
    [0.12, 0.72, 0.28],   # 3 绿
    [0.18, 0.32, 0.92],   # 4 蓝
    [0.55, 0.20, 0.85],   # 5 紫
    [0.88, 0.12, 0.78],   # 6 品红
    [0.95, 0.45, 0.62],   # 7 粉
    [0.72, 0.55, 0.32],   # 8 棕
    [0.20, 0.85, 0.50],   # 9 春绿
], dtype=np.float64)

N_LM = len(PALETTE)

PIPE_COLOR = np.array([0.98, 0.48, 0.05])   # 工程橙，海底管线常见涂装


def _catmull_rom(ctrl, samples_per_seg=40):
    """Catmull-Rom 样条：控制点 → 稠密折线。"""
    pts = np.asarray(ctrl, dtype=np.float64)
    if len(pts) < 3:
        t = np.linspace(0, 1, samples_per_seg)[:, None]
        return pts[0] + (pts[1] - pts[0]) * t
    out = []
    n = len(pts)
    for i in range(n - 1):
        p0 = pts[max(0, i - 1)]
        p1, p2 = pts[i], pts[i + 1]
        p3 = pts[min(n - 1, i + 2)]
        t = np.linspace(0.0, 1.0, samples_per_seg, endpoint=False)[:, None]
        a = 2.0 * p1
        b = -p0 + p2
        c = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3
        d = -p0 + 3.0 * p1 - 3.0 * p2 + p3
        seg = 0.5 * (a + b * t + c * t * t + d * t ** 3)
        out.append(seg)
    out.append(pts[-1][None, :])
    line = np.vstack(out)
    # 去掉重复点
    keep = np.ones(len(line), dtype=bool)
    keep[1:] = np.hypot(np.diff(line[:, 0]), np.diff(line[:, 1])) > 1e-6
    return line[keep]


def _seg_dist_px(o, d, p1, p2, r):
    """射线束到线段胶囊（中心线 p1-p2、半径 r）的最近命中距离。

    o:(2,) 起点；d:(K,2) 单位方向。返回 (K,)，inf 表示未命中。
    把 |o+td - (p1+s·eh)|² 对 t、s 联合最小化：s clamp 到 [0,L] 后
    由正交条件 t = (m·d) + s·(eh·d) 直接给出，无需解方程组。
    """
    m = np.asarray(p1, dtype=np.float64) - o                     # (2,)
    eh = np.asarray(p2, dtype=np.float64) - np.asarray(p1, dtype=np.float64)
    L = float(np.hypot(eh[0], eh[1]))
    if L < 1e-9:
        return np.full(len(d), np.inf)
    eh = eh / L
    B = d @ eh                                   # (K,) eh·d
    D = d @ m                                    # (K,) m·d
    # 最小化：t - s·B = D → t = D + s·B；∂/∂s=0 处 s* = (E + B·D)/(1-B²)
    E = float(eh @ m)
    den = np.maximum(1.0 - B * B, 1e-6)
    s = (E + B * D) / den
    s = np.clip(s, 0.0, L)
    t = D + B * s
    t = np.maximum(t, 0.0)
    qx = o[0] + t * d[:, 0] - (p1[0] + s * eh[0])
    qy = o[1] + t * d[:, 1] - (p1[1] + s * eh[1])
    d2 = qx * qx + qy * qy
    hit = (d2 <= r * r) & (t > 1e-6)
    return np.where(hit, t, np.inf)


class World:
    """海底管道 + 圆柱应答器 + 海流。"""

    def __init__(self, ctrl_pts, landmarks, start, current=None,
                 turbid=False, pipe_radius=None):
        self.ctrl = np.asarray(ctrl_pts, dtype=np.float64).reshape(-1, 2)
        self.pipeline = _catmull_rom(ctrl_pts)              # (N,2) 稠密折线
        self.landmarks = np.asarray(landmarks, dtype=np.float64).reshape(-1, 3)
        self.start = np.asarray(start, dtype=np.float64)
        self.pipe_r = P.PIPE_RADIUS if pipe_radius is None else float(pipe_radius)
        # current: (amp, omega, phase, dirx, diry) —— 幅值正弦摆动的定向流
        self.current = None if current is None else np.asarray(current, dtype=np.float64)
        self.turbid = bool(turbid)
        self.t = 0.0
        # 管道累计弧长（进度与真值评测用）
        seg = np.hypot(np.diff(self.pipeline[:, 0]), np.diff(self.pipeline[:, 1]))
        self.arc = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.arc[-1])
        self.end_a = self.pipeline[0].copy()
        self.end_b = self.pipeline[-1].copy()
        # 任务阶段：go = A→B，return = B→A
        self.phase = 'go'
        self.reached_b = False
        self.done = False

    # ------------------------------------------------------------ 基本量
    @property
    def goal(self):
        """当前任务目标点：去程是管道末端 B，回程是首端 A。"""
        return self.end_b if self.phase == 'go' else self.end_a

    def current_vector(self):
        if self.current is None:
            return np.zeros(2)
        amp, omega, phase, dx, dy = self.current
        c = amp * (0.6 + 0.4 * math.sin(omega * self.t + phase))
        n = math.hypot(dx, dy)
        return np.array([dx, dy]) / n * c

    def circles(self):
        """所有圆形物体 (M,3) = (x, y, r)：应答器。"""
        lm = self.landmarks[:, :2]
        r = np.full((len(lm), 1), P.LM_RADIUS)
        return np.hstack([lm, r])

    def progress_of(self, x, y):
        """点到管道的最近弧长位置与横向有符号距离（左正）。"""
        d2 = (self.pipeline[:, 0] - x) ** 2 + (self.pipeline[:, 1] - y) ** 2
        i = int(np.argmin(d2))
        px, py = self.pipeline[i]
        # 切向（按弧长前进方向）
        j = min(i + 1, len(self.pipeline) - 1)
        k = max(i - 1, 0)
        tx, ty = self.pipeline[j] - self.pipeline[k]
        tl = math.hypot(tx, ty) or 1.0
        tx, ty = tx / tl, ty / tl
        nx, ny = -ty, tx                      # 左法向
        s = float(self.arc[i] + ((x - px) * tx + (y - py) * ty))
        lat = float((x - px) * nx + (y - py) * ny)
        return s, lat, (tx, ty)

    def advance(self, pose):
        """阶段推进（按弧长进度，而不是端点距离——巡检偏置 0.9 m 时
        ROV 永远到不了端点本身）。返回 True 表示任务完成。"""
        if self.done:
            return True
        s, _lat, _t = self.progress_of(pose[0], pose[1])
        if self.phase == 'go' and s >= self.length - 0.8:
            self.phase = 'return'
        elif self.phase == 'return' and s <= 0.8:
            self.done = True
            return True
        return False

    def mission_direction(self, x, y):
        """巡检方向（go 沿弧长正向，return 反向），供控制器对齐切向。"""
        return 1.0 if self.phase == 'go' else -1.0

    def step(self, dt):
        self.t += dt

    def start_pose(self, dlat=0.0, dth=0.0):
        """起点在偏置巡检线上的横向/艏向偏置版本。

        数据采集要用它做**起始扰动**：专家轨迹本身横向误差几乎恒为 0，
        只用这种数据训出来的回归器会退化成恒输出 0（均值坍缩）——
        部署时哪怕真的偏了 1.8 m，它也报"没偏"。
        """
        n0 = _left_normal(self.pipeline)[0]
        t0 = self.pipeline[1] - self.pipeline[0]
        t0 = t0 / (np.hypot(t0[0], t0[1]) or 1.0)
        base = self.end_a[0] + n0[0] * P.FOLLOW_OFFSET
        basey = self.end_a[1] + n0[1] * P.FOLLOW_OFFSET
        return np.array([base + n0[0] * dlat, basey + n0[1] * dlat,
                         math.atan2(t0[1], t0[0]) + dth])

    # ------------------------------------------------------------ 求交
    def raycast(self, origin, angles, max_range=None):
        """对一批射线求最近命中。

        返回 (dist, hid)：hid=0 未命中；1..N 为应答器索引+1；
        N+1 为管道。管道返回的是胶囊表面的距离。
        """
        max_range = P.CAM_RANGE_MAX if max_range is None else max_range
        ox, oy = float(origin[0]), float(origin[1])
        ang = np.asarray(angles, dtype=np.float64).reshape(-1)
        dx, dy = np.cos(ang), np.sin(ang)
        K = ang.size

        best = np.full(K, max_range, dtype=np.float64)
        hid = np.zeros(K, dtype=np.int32)

        # --- 圆柱应答器（二次方程取近根）
        lm = self.landmarks[:, :2]
        if len(lm):
            cx = lm[:, 0][None, :] - ox
            cy = lm[:, 1][None, :] - oy
            r = np.full((1, len(lm)), P.LM_RADIUS)
            b = cx * dx[:, None] + cy * dy[:, None]
            c0 = cx * cx + cy * cy - (r ** 2)
            disc = b * b - c0
            ok = disc > 0
            sq = np.sqrt(np.where(ok, disc, 0.0))
            t = b - sq
            t = np.where(ok & (t > 1e-6), t, np.inf)
            idx = np.argmin(t, axis=1)
            tb = t[np.arange(K), idx]
            m = tb < best
            best[m] = tb[m]
            hid[m] = (idx[m] + 1).astype(np.int32)

        # --- 管道：逐段胶囊求交
        segs = self.pipeline
        if len(segs) > 1:
            for i in range(len(segs) - 1):
                t = _seg_dist_px((ox, oy), np.stack([dx, dy], axis=1),
                                 segs[i], segs[i + 1], self.pipe_r)
                m = t < best
                best[m] = t[m]
                hid[m] = len(lm) + 1

        return best, hid

    # ------------------------------------------------------------ 占用查询
    def point_free(self, x, y, margin=0.0):
        """点是否在自由空间（含 ROV 外扩 margin）。压管/撞应答器/出界都不行。"""
        if not (-margin < x < P.WORLD_W + margin and -margin < y < P.WORLD_H + margin):
            return False
        for cx, cy, _cid in self.landmarks:
            if (x - cx) ** 2 + (y - cy) ** 2 <= (P.LM_RADIUS + margin) ** 2:
                return False
        # 管道胶囊
        segs = self.pipeline
        px, py = x, y
        for i in range(len(segs) - 1):
            ax, ay = segs[i]
            bx, by = segs[i + 1]
            ex, ey = bx - ax, by - ay
            L2 = ex * ex + ey * ey
            t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0,
                ((px - ax) * ex + (py - ay) * ey) / L2))
            qx, qy = ax + t * ex - px, ay + t * ey - py
            if qx * qx + qy * qy <= (self.pipe_r + margin) ** 2:
                return False
        return True


def _layout_features(key, rng):
    """应答器：沿管道两侧 1.8~2.8 m 分布，保证巡检途中持续有观测。

    偏置巡检路径（管道 + FOLLOW_OFFSET 的左法向）上不能有应答器，
    否则场景在真值上就不可行 —— S 弯的另一段管道会绕到路径附近，
    布点时按"距参考路径 > 0.6 m"重采样。
    """
    ref = _catmull_rom(_ctrl(key, np.random.default_rng(0)))
    guide = ref + _left_normal(ref) * P.FOLLOW_OFFSET

    def clear_of_guide(x, y):
        d = np.hypot(guide[:, 0] - x, guide[:, 1] - y)
        # 净空必须盖住：跟踪横向误差（LAT_OK=0.45）+ ROV 半径 + 折返
        # 切角余量。0.6 的旧值放行了距巡检路径仅 0.96 m 的点，折返时
        # 必撞；1.05 m 保证真值意义上全程无碰。
        return float(np.min(d)) > 1.05

    out = []
    for lid in range(N_LM):
        s = (lid + 0.5) / float(N_LM)
        idx = int(s * (len(ref) - 1))
        px, py = ref[idx]
        j = min(idx + 1, len(ref) - 1)
        k = max(idx - 1, 0)
        tx, ty = ref[j] - ref[k]
        tl = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / tl, tx / tl
        fx = fy = None
        for attempt in range(120):
            # 优先交替侧，多次失败后允许换侧：巡检路径两侧都可以布点
            side = (1.0 if lid % 2 == 0 else -1.0) * (1 if attempt < 60 else -1)
            d = rng.uniform(1.8, 2.8) * side
            cx = px + nx * d + rng.normal(0, 0.08)
            cy = py + ny * d + rng.normal(0, 0.08)
            if not clear_of_guide(cx, cy):
                continue
            if not (0.5 < cx < P.WORLD_W - 0.5 and 0.5 < cy < P.WORLD_H - 0.5):
                continue
            if any(math.hypot(cx - ox, cy - oy) < 0.9 for ox, oy, _ in out):
                continue
            fx, fy = cx, cy
            break
        if fx is None:
            fx, fy = px + nx * 2.6, py + ny * 2.6
        out.append((fx, fy, lid))
    return out


def _ctrl(key, rng):
    """各场景的管道控制点。"""
    if key in ('straight', 'current'):
        pts = [(3.0, 8.0), (12.0, 8.0), (21.0, 8.0)]
        if key == 'straight':
            pts = [(x + rng.normal(0, 0.05), y + rng.normal(0, 0.05))
                   for x, y in pts]
        return pts
    if key in ('curve', 'turbid'):
        return [(3.0, 8.0), (8.0, 10.5), (12.0, 6.0), (16.5, 10.0), (21.0, 8.0)]
    raise ValueError('unknown scene: %s' % key)


def make_scene(key, seed=0):
    """构造四个场景之一。起终点一致，差异在管道形状/海流/浑浊度。"""
    rng = np.random.default_rng(seed * 1000 + 7)
    current = None
    turbid = False

    if key == 'straight':
        ctrl = _ctrl('straight', rng)
    elif key == 'curve':
        ctrl = _ctrl('curve', rng)
    elif key == 'current':
        ctrl = _ctrl('current', rng)
        # 横向流为主（垂直管道），叠加少量顺流：里程计完全感知不到它
        amp = P.CURRENT_MAX * (0.9 + 0.2 * rng.uniform())
        current = (amp, P.CURRENT_OMEGA, rng.uniform(0, 6.28), 0.15, 1.0)
    elif key == 'turbid':
        ctrl = _ctrl('turbid', rng)
        turbid = True
    else:
        raise ValueError('unknown scene: %s' % key)

    feats = _layout_features(key, rng)
    w = World(ctrl, feats, start=(ctrl[0][0], ctrl[0][1], 0.0),
              current=current, turbid=turbid)
    # 起点放在偏置巡检路径上（管道左侧 FOLLOW_OFFSET），朝向沿巡检切向
    w.start = w.start_pose()
    w.key = key
    return w


def _reachable(w):
    """管道两侧偏置路径在真值上全程无碰（应答器不挡路、管不被压）。"""
    line = w.pipeline
    n = w.pipeline + _left_normal(line) * P.FOLLOW_OFFSET
    for x, y in n:
        if not w.point_free(x, y, margin=P.ROBOT_RADIUS):
            return False
    return True


def _left_normal(line):
    out = np.zeros_like(line)
    t = np.vstack([line[1] - line[0], line[2:] - line[:-2], line[-1] - line[-2]])
    L = np.hypot(t[:, 0], t[:, 1])
    L[L < 1e-9] = 1.0
    out[:, 0] = -t[:, 1] / L
    out[:, 1] = t[:, 0] / L
    return out
