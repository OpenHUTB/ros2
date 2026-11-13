"""点到点速度规划：梯形速度曲线与 S 曲线（jerk 限制），纯标准库。

两种曲线解决同一个问题：把负载从静止移动到距离 d，速度上限 vmax、加速度
上限 amax，要求末端速度精确为 0。

梯形（Trapezoidal / "T 型"）
    加速度是**方波**：0 → +amax → 0 → -amax → 0。
    时间最短，但加速度在切换点跳变，意味着 jerk 为无穷大（冲击、抖动、
    机械磨损、液体晃动）。工程上常被称为"速度环阶跃"。

S 曲线（Jerk-limited）
    加速度是**梯形**：jerk 先 +jmax、再 0、再 -jmax，使加速度平滑地升到 amax。
    代价是耗时更长，但 jerk 有界，运动平滑，是 CNC / 机械臂 / 电梯的标配。

本实现的 S 曲线用"分段解析 jerk 积分"构造：先算出加速段时长，再对 jerk
做数值积分得到 v(t)、s(t)；若距离不足，用二分法求可行的峰值速度。
"""

import math


# ---------------------------------------------------------------------------
# 梯形速度曲线（解析解）
# ---------------------------------------------------------------------------


class TrapezoidProfile:
    def __init__(self, distance, vmax, amax):
        assert distance > 0 and vmax > 0 and amax > 0
        self.d = float(distance)
        self.vmax = float(vmax)
        self.amax = float(amax)
        self.triangular = False
        d_reach = self.vmax * self.vmax / self.amax   # 加速+减速所需距离
        if self.d >= d_reach:
            self.vpeak = self.vmax
            self.ta = self.vmax / self.amax
            self.tc = (self.d - d_reach) / self.vmax
        else:
            # 距离太短，冲不到 vmax，形成三角形
            self.triangular = True
            self.vpeak = math.sqrt(self.amax * self.d)
            self.ta = self.vpeak / self.amax
            self.tc = 0.0
        self.td = self.ta
        self.T = self.ta + self.tc + self.td

    def eval(self, t):
        """返回 (s, v, a)。"""
        if t <= 0:
            return 0.0, 0.0, 0.0
        if t >= self.T:
            return self.d, 0.0, 0.0
        if t < self.ta:                                  # 加速
            v = self.amax * t
            return 0.5 * self.amax * t * t, v, self.amax
        if t < self.ta + self.tc:                        # 匀速
            s = 0.5 * self.amax * self.ta ** 2 + self.vpeak * (t - self.ta)
            return s, self.vpeak, 0.0
        # 减速
        t2 = t - self.ta - self.tc
        v = self.vpeak - self.amax * t2
        s = (0.5 * self.amax * self.ta ** 2 + self.vpeak * self.tc
             + self.vpeak * t2 - 0.5 * self.amax * t2 * t2)
        return s, max(v, 0.0), -self.amax

    def samples(self, dt=None):
        n = int(math.ceil(self.T / (dt or self.T / 400.0)))
        ts = [self.T * i / n for i in range(n + 1)]
        out = [self.eval(t) for t in ts]
        return ts, [o[0] for o in out], [o[1] for o in out], [o[2] for o in out]


# ---------------------------------------------------------------------------
# S 曲线（jerk 限制）
# ---------------------------------------------------------------------------


def _s_accel(vpeak, amax, jmax):
    """从 0 加速到 vpeak 的最短时间（受 amax、jmax 双重限制）。"""
    if vpeak >= amax * amax / jmax:
        tj = amax / jmax
        ta = vpeak / amax - tj
        return tj, ta
    # 加速度三角形：达不到 amax
    tj = math.sqrt(vpeak / jmax)
    return tj, 0.0


def _s_accel_distance(vpeak, amax, jmax):
    tj, ta = _s_accel(vpeak, amax, jmax)
    t_total = 2.0 * tj + ta
    # 速度曲线关于中点对称 -> 平均速度 = vpeak/2
    return vpeak * t_total / 2.0


class SCurveProfile:
    """jerk 受限的 S 曲线（起止速度均为 0）。"""

    def __init__(self, distance, vmax, amax, jmax, dt=2e-4):
        assert distance > 0 and vmax > 0 and amax > 0 and jmax > 0
        self.d = float(distance)
        self.vmax = float(vmax)
        self.amax = float(amax)
        self.jmax = float(jmax)

        # 1) 先看能否冲到 vmax：加速段 + 减速段距离 <= d
        d_acc_max = _s_accel_distance(vmax, amax, jmax)
        if 2.0 * d_acc_max <= self.d + 1e-12:
            self.vpeak = vmax
        else:
            # 2) 二分求可行峰值速度：2*d_acc(v) = d
            lo, hi = 1e-9, vmax
            for _ in range(200):
                mid = 0.5 * (lo + hi)
                if 2.0 * _s_accel_distance(mid, amax, jmax) <= self.d:
                    lo = mid
                else:
                    hi = mid
            self.vpeak = lo

        self.tj, self.ta = _s_accel(self.vpeak, amax, jmax)
        self.t_acc = 2.0 * self.tj + self.ta
        d_acc = _s_accel_distance(self.vpeak, amax, jmax)
        self.tc = max((self.d - 2.0 * d_acc) / self.vpeak, 0.0)
        self.T = 2.0 * self.t_acc + self.tc

        # 3) 积分 jerk 得到 s(t), v(t), a(t)
        self._build(dt)

    def _jerk_profile(self):
        tj, ta, tc = self.tj, self.ta, self.tc
        # (时长, jerk)：加速(+j,0,-j) -> 匀速 -> 减速(-j,0,+j)
        return [(tj, self.jmax), (ta, 0.0), (tj, -self.jmax),
                (tc, 0.0),
                (tj, -self.jmax), (ta, 0.0), (tj, self.jmax)]

    def _build(self, dt):
        ts, ss, vs, as_, js = [0.0], [0.0], [0.0], [0.0], [0.0]
        s = v = a = 0.0
        t = 0.0
        for dur, jerk in self._jerk_profile():
            if dur <= 0:
                continue
            n = max(1, int(math.ceil(dur / dt)))
            h = dur / n
            for _ in range(n):
                # 中点法积分，保证二阶精度
                a_new = a + jerk * h
                v_mid = v + 0.5 * (a + a_new) * h
                s += v_mid * h
                v += 0.5 * (a + a_new) * h
                a = a_new
                t += h
                ts.append(t)
                ss.append(s)
                vs.append(v)
                as_.append(a)
                js.append(jerk)
        # 用解析总距离做标定，消除数值积分残差
        if ss[-1] > 0:
            k = self.d / ss[-1]
            ss = [x * k for x in ss]
        self.ts, self.ss, self.vs, self.as_, self.js = ts, ss, vs, as_, js

    def eval(self, t):
        if t <= 0:
            return 0.0, 0.0, 0.0
        if t >= self.T:
            return self.d, 0.0, 0.0
        i = min(int(t / self.T * (len(self.ts) - 1)), len(self.ts) - 2)
        while i + 1 < len(self.ts) and self.ts[i + 1] < t:
            i += 1
        return self.ss[i], self.vs[i], self.as_[i]

    def samples(self):
        return self.ts, self.ss, self.vs, self.as_

    # -- 指标 --------------------------------------------------------------
    def peak_jerk(self):
        return max(abs(x) for x in self.js)

    def peak_accel(self):
        return max(abs(x) for x in self.as_)

    def peak_velocity(self):
        return max(self.vs)
