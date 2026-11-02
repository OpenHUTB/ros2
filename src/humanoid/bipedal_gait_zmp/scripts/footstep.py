# -*- coding: utf-8 -*-
"""落脚点规划、支撑多边形与 ZMP 参考轨迹。

坐标系只考虑前后方向（x）：机器人沿 +x 行走，左右脚的差别在这个一维模型里
不体现，只体现为"支撑区域在 x 轴上的区间"。

步态时序
--------
每一步包含两段：

    单脚支撑（single support）  只有一只脚着地，支撑区间就是这只脚的脚掌范围
    双脚支撑（double support）  两只脚同时着地，支撑区间是两脚脚掌的并集

ZMP 参考取值
------------
* 单脚支撑时取支撑脚脚掌中心
* 双脚支撑时从上一只脚的中心线性过渡到下一只脚的中心

只要每一时刻 ZMP 都落在支撑区间内，这个步态在 ZMP 判据下就是稳定的。
"""

from __future__ import annotations

__all__ = ["Footstep", "GaitPlan", "support_polygon", "stability_margin"]


class Footstep(object):
    """一次落脚。

    参数
    ----
    side       'L' 或 'R'
    center     脚掌中心的 x 坐标 (m)
    t_start    这只脚开始着地的时刻 (s)
    t_end      这只脚离地的时刻 (s)
    half_length 脚掌前后半长 (m)，用来构造支撑区间
    """

    def __init__(self, side, center, t_start, t_end, half_length=0.12):
        if side not in ("L", "R"):
            raise ValueError("side 只能是 'L' 或 'R'")
        if t_end <= t_start:
            raise ValueError("t_end 必须大于 t_start")
        if half_length <= 0:
            raise ValueError("half_length 必须为正")

        self.side = side
        self.center = float(center)
        self.t_start = float(t_start)
        self.t_end = float(t_end)
        self.half_length = float(half_length)

    @property
    def reach(self):
        """脚掌在 x 轴上的区间 (lo, hi)。"""
        return (self.center - self.half_length,
                self.center + self.half_length)

    def covers(self, t):
        return self.t_start <= t <= self.t_end

    def __repr__(self):
        return "Footstep(%s, x=%.3f, %.2f~%.2f)" % (
            self.side, self.center, self.t_start, self.t_end)


class GaitPlan(object):
    """一串落脚点按时间排好后形成的步态计划。"""

    def __init__(self, footsteps):
        if not footsteps:
            raise ValueError("footsteps 不能为空")
        steps = sorted(footsteps, key=lambda f: f.t_start)
        for a, b in zip(steps, steps[1:]):
            if b.t_start < a.t_start:
                raise ValueError("落脚点顺序异常")
        self.steps = steps

    @property
    def t_start(self):
        return self.steps[0].t_start

    @property
    def t_end(self):
        return max(s.t_end for s in self.steps)

    @property
    def duration(self):
        return self.t_end - self.t_start

    def feet_at(self, t):
        """返回 t 时刻着地的脚（0、1 或 2 只）。"""
        return [s for s in self.steps if s.covers(t)]

    def support_polygon(self, t):
        """t 时刻的支撑区间 (lo, hi)。

        t 超出整个计划范围时返回最后一只脚的区间（便于边界处理）。
        """
        on = self.feet_at(t)
        if not on:
            if t < self.t_start:
                return self.steps[0].reach
            return self.steps[-1].reach
        lo = min(s.reach[0] for s in on)
        hi = max(s.reach[1] for s in on)
        return (lo, hi)

    def zmp_reference(self, t):
        """t 时刻的 ZMP 参考值。

        单脚支撑：支撑脚中心。
        双脚支撑：从最早落地的脚中心线性过渡到最晚落地的脚中心。
        """
        on = self.feet_at(t)
        if not on:
            if t < self.t_start:
                return self.steps[0].center
            return self.steps[-1].center

        if len(on) == 1:
            return on[0].center

        # 双脚支撑：找到这次过渡的起止时间
        first = min(on, key=lambda s: s.t_start)
        second = max(on, key=lambda s: s.t_start)
        span = second.t_start - first.t_start
        if span <= 0:
            return second.center
        ratio = (t - second.t_start) / (first.t_end - second.t_start) \
            if first.t_end > second.t_start else 1.0
        ratio = max(0.0, min(1.0, ratio))
        return first.center + (second.center - first.center) * ratio

    def phase_kind(self, t):
        """返回 t 时刻的阶段类型：'single' / 'double' / 'none'。"""
        n = len(self.feet_at(t))
        if n == 0:
            return "none"
        return "single" if n == 1 else "double"

    def __repr__(self):
        return "GaitPlan(%d steps, %.2f~%.2f s)" % (
            len(self.steps), self.t_start, self.t_end)


# ---------------------------------------------------------------------------
def support_polygon(plan, t):
    """便捷函数：等价于 ``plan.support_polygon(t)``。"""
    return plan.support_polygon(t)


def stability_margin(zmp, polygon):
    """ZMP 到支撑区间边界的最小距离 (m)。

    值为正表示 ZMP 在区间内，越接近 0 越危险；为负表示已经越界（失稳）。
    """
    lo, hi = polygon
    if lo > hi:
        raise ValueError("支撑区间非法: lo > hi")
    return min(zmp - lo, hi - zmp)
