"""Dubins 最短路径：满足最小转弯半径约束的两点位姿间最短曲线。

车辆不能原地转向，也走不出比 ``R_min`` 更急的弯，因此两点之间最短的曲线
不再是直线，而是「圆弧 + 直线 + 圆弧」或「弧-弧-弧」的组合。Dubins 证明
最优解必属于六种基本情形：

    LSL  RSR  LSR  RSL   （弧-直线-弧）
    RLR  LRL              （弧-弧-弧）

L 表示左转弧（逆时针），R 表示右转弧（顺时针），S 表示直线。

实现方式：不套用容易写错的闭式公式，而是直接做几何构造——先求起点圆、
终点圆的圆心，再求两圆的外/内公切线方向与公切圆圆心，最后**用端点闭环
校验筛选候选解**。这样每个候选都自带正确性证明：转成三段弧长后积分，
必须精确回到目标位姿，否则该分支直接丢弃。

坐标约定：x 向右、y 向上，航向 theta 从 x 轴逆时针度量，单位均为米 / 弧度。
"""

import math

TWO_PI = 2.0 * math.pi

# 段类型 -> 归一化曲率符号（乘 1/R 得实际曲率；S 段为 0）
_KAPPA_SIGN = {'L': 1.0, 'R': -1.0, 'S': 0.0}

_CASES = ('LSL', 'LSR', 'RSL', 'RSR', 'RLR', 'LRL')

# 端点校验容差（米 / 弧度）
_TOL_POS = 1e-7
_TOL_ANG = 1e-7


def mod2pi(x):
    """把角度折回 [0, 2*pi)。"""
    x = math.fmod(x, TWO_PI)
    if x < 0.0:
        x += TWO_PI
    return x


def wrap_pi(a):
    """把角度折回 (-pi, pi]。"""
    w = math.fmod(a, TWO_PI)
    if w > math.pi:
        w -= TWO_PI
    elif w <= -math.pi:
        w += TWO_PI
    return w


def _center(pose, kind, radius):
    """起/终点所在圆弧的圆心（kind 为 'L' 时圆心在左法向）。"""
    x, y, th = pose[0], pose[1], pose[2]
    if kind == 'L':
        return (x - radius * math.sin(th), y + radius * math.cos(th))
    if kind == 'R':
        return (x + radius * math.sin(th), y - radius * math.cos(th))
    raise ValueError("只有 L / R 段才有圆心")


def _arc_angle(center, p_from, p_to, kind):
    """绕圆心从 p_from 转到 p_to 的转角，方向由 kind 决定。"""
    a1 = math.atan2(p_from[1] - center[1], p_from[0] - center[0])
    a2 = math.atan2(p_to[1] - center[1], p_to[0] - center[0])
    return mod2pi(a2 - a1) if kind == 'L' else mod2pi(a1 - a2)


class DubinsPath(object):
    """一条 Dubins 路径：segments 为 [(类型, 长度), ...]，最多三段。"""

    def __init__(self, start, radius, segments, case):
        self.start = tuple(start)          # (x, y, theta)
        self.radius = float(radius)
        self.segments = list(segments)     # [('L', len), ('S', len), ...]
        self.case = case

    @property
    def length(self):
        return sum(seg[1] for seg in self.segments)

    def curvature(self, kind):
        return _KAPPA_SIGN[kind] / self.radius

    # -- 单段推进：用解析解，避免小步长积分的累计误差 --------------------
    def _advance(self, pose, kind, seg_len):
        x, y, th = pose[0], pose[1], pose[2]
        kappa = self.curvature(kind)
        if kappa == 0.0:
            return (x + seg_len * math.cos(th),
                    y + seg_len * math.sin(th), th)
        dth = kappa * seg_len
        r = 1.0 / abs(kappa)
        sign = 1.0 if kappa > 0.0 else -1.0
        return (x + sign * r * (math.sin(th + dth) - math.sin(th)),
                y + sign * r * (math.cos(th) - math.cos(th + dth)),
                th + dth)

    def end_pose(self):
        x, y, th = self.start
        for kind, seg_len in self.segments:
            x, y, th = self._advance((x, y, th), kind, seg_len)
        return x, y, th

    def seg_starts(self):
        """各段起点的位姿（首项为整条路径起点）。"""
        out = [tuple(self.start)]
        pose = tuple(self.start)
        for kind, seg_len in self.segments:
            pose = self._advance(pose, kind, seg_len)
            out.append(pose)
        return out

    def pose_at(self, s):
        """按弧长 s 取位姿 (x, y, theta, kappa)，s 自动截断到 [0, 总长]。"""
        s = min(max(s, 0.0), self.length)
        acc = 0.0
        pose = tuple(self.start)
        for kind, seg_len in self.segments:
            if seg_len <= 0.0:
                continue
            if s <= acc + seg_len + 1e-12:
                local = min(max(s - acc, 0.0), seg_len)
                p = self._advance(pose, kind, local)
                return (p[0], p[1], p[2], self.curvature(kind))
            acc += seg_len
            pose = self._advance(pose, kind, seg_len)
        return (pose[0], pose[1], pose[2], 0.0)

    def sample(self, step=0.05):
        """按弧长等间隔采样，返回 [(x, y, theta, kappa), ...]。"""
        n = max(2, int(math.ceil(self.length / max(step, 1e-9))))
        return [self.pose_at(self.length * i / n) for i in range(n + 1)]

    def samples_at(self, s_values):
        return [self.pose_at(s) for s in s_values]

    def endpoint_error(self, goal):
        ex, ey, eth = self.end_pose()
        return (math.hypot(ex - goal[0], ey - goal[1]),
                abs(wrap_pi(eth - goal[2])))

    def __repr__(self):
        segs = " ".join("%s%.3f" % (k, l) for k, l in self.segments)
        return "<DubinsPath %s len=%.4f %s>" % (self.case, self.length, segs)


def _candidates_straight(start, goal, radius, case):
    """弧-直线-弧：用两圆的外/内公切线求方向与直线长度。"""
    k1, _, k3 = case
    c1 = _center(start, k1, radius)
    c3 = _center(goal, k3, radius)
    dx, dy = c3[0] - c1[0], c3[1] - c1[1]
    dist = math.hypot(dx, dy)

    if dist < 1e-9:
        # 两圆心重合：只有同向且终点就在起点圆上的退化情形
        if k1 != k3:
            return
        t = _arc_angle(c1, start, goal, k1)
        yield DubinsPath(start, radius, [(k1, t * radius), ('S', 0.0),
                                         (k3, 0.0)], case)
        return

    base = math.atan2(dy, dx)
    if k1 == k3:
        # 同向转：外公切线，方向平行于圆心连线，长度等于圆心距
        dirs = [base]
        seg_len = dist
    else:
        # 反向转：内公切线，需要两圆相离（圆心距 >= 2R）
        if dist < 2.0 * radius - 1e-9:
            return
        seg_len = math.sqrt(max(0.0, dist * dist - 4.0 * radius * radius))
        if seg_len < 1e-9:
            gamma = math.pi / 2.0
        else:
            gamma = math.atan2(2.0 * radius, seg_len)
        dirs = [base + gamma, base - gamma]

    sign1 = 1.0 if k1 == 'L' else -1.0
    sign3 = 1.0 if k3 == 'L' else -1.0
    for u_ang in dirs:
        t = mod2pi((u_ang - start[2]) * sign1)
        q = mod2pi((goal[2] - u_ang) * sign3)
        yield DubinsPath(start, radius,
                         [(k1, t * radius), ('S', seg_len), (k3, q * radius)],
                         case)


def _candidates_arc(start, goal, radius, case):
    """弧-弧-弧（RLR / LRL）：求与两个端圆都外切的中间圆。"""
    k1, km, k3 = case
    c1 = _center(start, k1, radius)
    c3 = _center(goal, k3, radius)
    dx, dy = c3[0] - c1[0], c3[1] - c1[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9 or dist > 4.0 * radius + 1e-9:
        return
    ux, uy = dx / dist, dy / dist
    mx, my = (c1[0] + c3[0]) / 2.0, (c1[1] + c3[1]) / 2.0
    h = math.sqrt(max(0.0, 4.0 * radius * radius - dist * dist / 4.0))
    for s in (1.0, -1.0):
        c2 = (mx - s * h * uy, my + s * h * ux)
        p1 = ((c1[0] + c2[0]) / 2.0, (c1[1] + c2[1]) / 2.0)   # 两圆外切点
        p2 = ((c2[0] + c3[0]) / 2.0, (c2[1] + c3[1]) / 2.0)
        t = _arc_angle(c1, start, p1, k1)
        p = _arc_angle(c2, p1, p2, km)
        q = _arc_angle(c3, p2, goal, k3)
        yield DubinsPath(start, radius,
                         [(k1, t * radius), (km, p * radius),
                          (k3, q * radius)], case)


def all_paths(start, goal, radius, check=True):
    """返回全部可行的候选路径（按长度升序）。

    check=True 时用端点闭环校验过滤掉几何分支的伪解。
    """
    if radius <= 0.0:
        raise ValueError("最小转弯半径必须为正")
    out = []
    for case in _CASES:
        gen = (_candidates_arc if case[1] != 'S' else _candidates_straight)
        for path in gen(start, goal, radius, case):
            if any(seg[1] < -1e-12 for seg in path.segments):
                continue
            if check:
                dp, dth = path.endpoint_error(goal)
                if dp > _TOL_POS or dth > _TOL_ANG:
                    continue
            out.append(path)
    out.sort(key=lambda p: p.length)
    return out


def dubins_path(start, goal, radius):
    """求最短 Dubins 路径。

    start / goal: (x, y, theta)；radius: 最小转弯半径（米）。
    返回 DubinsPath；无可行解时抛 ValueError。
    """
    paths = all_paths(start, goal, radius)
    if not paths:
        raise ValueError("给定位姿与转弯半径下无可行 Dubins 路径")
    return paths[0]
