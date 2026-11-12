"""过渡走廊：在 (空速, 倾转角) 平面上，平飞约束能同时满足的区域。

平飞要求两个方程成立：

.. math::

    T \\sin\\delta + L = W, \\qquad T \\cos\\delta \\ge D(C_L)

第二个是不等式而不是等式：过渡过程中允许有净推力用于加速，只要**推得出**就行。
垂直方向必须严格平衡，否则就是在爬升或掉高度。

``(V, δ)`` 固定后未知量只有 ``T`` 与 ``C_L``：垂直平衡给出 ``T = (W - L)/sinδ``，
于是 ``C_L`` 成为唯一自由度，逐点扫描即可判定该 (V, δ) 是否可行
（同时满足 ``C_L ∈ [C_Lmin, C_Lmax]``、``T ≤ T_max(V)``、``T cosδ ≥ D``）。

走廊的两条边界由此分别产生：

* 低速端 ``δ ≥ δ_min(V)``：机翼升力不足，倾转角必须够"立"才托得住全重；
* 高速端 ``δ ≤ δ_max(V)``：``cosδ`` 太小，水平推力压不过阻力。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .vehicle import VehicleSpec

__all__ = ["LevelSolution", "Corridor"]


@dataclass(frozen=True)
class LevelSolution:
    """某一 (V, δ) 下的可行平飞平衡点（取功率最小的那种配平）。"""

    speed: float
    tilt_deg: float
    thrust: float
    cl: float
    drag: float
    power: float
    reserve: float          # T_max(V) - T，剩余推力（用于加速）

    @property
    def tilt_rad(self) -> float:
        return math.radians(self.tilt_deg)


class Corridor:
    """给定机型 + 高度，计算并查询过渡走廊。"""

    CL_SAMPLES = 161

    def __init__(self, spec: VehicleSpec | None = None, alt_m: float = 0.0):
        self.spec = spec or VehicleSpec()
        self.alt_m = alt_m
        self.rho = self.spec.density(alt_m)
        self._tilt_cache: dict[tuple[float, float], tuple[LevelSolution, ...]] = {}
        self._bounds_cache: dict[float, tuple[float, float] | None] = {}
        self._table: tuple[list[float], list[float], list[float]] | None = None
        self._tmax_table: tuple[list[float], list[float]] | None = None

    # ------------------------------------------------------------ 推力上限
    def thrust_max(self, v: float) -> float:
        """当前高度下的最大推力。

        逐点解 ``P(T) = P_max`` 要二分 80 次，控制回路每步都要用，太慢；
        这里按 0.1 m/s 建表线性插值（``T_max`` 随速度单调下降，插值误差 < 0.1%）。
        """
        if self._tmax_table is None:
            vs, ts = [], []
            v_ = 0.0
            while v_ <= 40.0:
                vs.append(v_)
                ts.append(self.spec.rotor.max_thrust(v_, self.rho))
                v_ += 0.1
            self._tmax_table = (vs, ts)
        vs, ts = self._tmax_table
        if v <= vs[0]:
            return ts[0]
        if v >= vs[-1]:
            return ts[-1]
        i = int(v / (vs[1] - vs[0]))
        f = (v - vs[i]) / (vs[i + 1] - vs[i])
        return ts[i] + f * (ts[i + 1] - ts[i])

    # ------------------------------------------------------------ 配平求解
    def trims(self, v: float, tilt_deg: float) -> tuple[LevelSolution, ...]:
        """返回该 ``(V, δ)`` 下可行配平（按功率升序，实际只返回最省功率的那个）。

        解算不用扫描：给定 ``(V, δ)`` 后 ``T = (W - qS·C_L)/sinδ`` 随 ``C_L`` 单调下降，
        而 ``D(C_L)`` 单调上升，于是可行性等价于一个区间：

        * ``T ≤ T_max``  →  ``C_L ≥ (W - T_max·sinδ) / qS``，给出下界 ``C_L_lo``；
        * ``T·cosδ ≥ D`` →  ``g(C_L) = T·cosδ - D ≥ 0``，``g`` 单调下降，二分得到上界。

        两个界之间可行，最省功率的解落在上界（``C_L`` 越大 → 推力越小 → 功率越小）。
        """
        key = (round(v, 4), round(tilt_deg, 3))
        cached = self._tilt_cache.get(key)
        if cached is not None:
            return cached

        spec, rho = self.spec, self.rho
        if not 0.0 <= tilt_deg <= 90.0:
            self._tilt_cache[key] = ()
            return ()

        d = math.radians(tilt_deg)
        sd, cd = math.sin(d), math.cos(d)
        w = spec.weight
        q_s = 0.5 * rho * v * v * spec.wing.area
        t_max = self.thrust_max(v)
        axial = v * abs(cd)

        if q_s <= 1e-9:
            # 悬停：升力为零，全重由旋翼承担
            if sd <= 1e-9 or w / sd > t_max + 1e-9:
                self._tilt_cache[key] = ()
                return ()
            t = w / sd
            sol = LevelSolution(v, tilt_deg, t, 0.0, 0.0,
                                spec.rotor.shaft_power(t, 0.0, rho), t_max - t)
            self._tilt_cache[key] = (sol,)
            return (sol,)

        if sd <= 1e-9:
            # δ=0°：旋翼不提供升力，机翼必须恰好撑住全重，推力全部用于克服阻力
            cl = w / q_s
            if not (spec.wing.cl_min <= cl <= spec.wing.cl_max):
                self._tilt_cache[key] = ()
                return ()
            drag = spec.drag_at(v, cl, rho)
            if drag > t_max:
                self._tilt_cache[key] = ()
                return ()
            sol = LevelSolution(v, tilt_deg, drag, cl, drag,
                                spec.rotor.shaft_power(drag, v, rho), t_max - drag)
            self._tilt_cache[key] = (sol,)
            return (sol,)

        cl_lo = (w - t_max * sd) / q_s
        cl_lo = max(cl_lo, spec.wing.cl_min)
        cl_hi = spec.wing.cl_max
        if cl_lo > cl_hi:
            self._tilt_cache[key] = ()
            return ()

        def g(cl: float) -> float:
            thrust = (w - q_s * cl) / sd if sd > 1e-9 else math.inf
            if thrust < 0.0:
                return -spec.drag_at(v, cl, rho)
            return thrust * cd - spec.drag_at(v, cl, rho)

        if g(cl_lo) < 0.0:
            self._tilt_cache[key] = ()
            return ()
        if g(cl_hi) >= 0.0:
            cl_best = cl_hi
        else:
            lo, hi = cl_lo, cl_hi
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if g(mid) >= 0.0:
                    lo = mid
                else:
                    hi = mid
            cl_best = lo

        thrust = (w - q_s * cl_best) / sd
        drag = spec.drag_at(v, cl_best, rho)
        sol = LevelSolution(v, tilt_deg, thrust, cl_best, drag,
                            spec.rotor.shaft_power(thrust, axial, rho), t_max - thrust)
        self._tilt_cache[key] = (sol,)
        return (sol,)

    def level_flight(self, v: float, tilt_deg: float) -> LevelSolution | None:
        """该 ``(V, δ)`` 下的最优（最省功率）平飞配平；不可行返回 ``None``。"""
        tr = self.trims(v, tilt_deg)
        return tr[0] if tr else None

    def feasible(self, v: float, tilt_deg: float) -> bool:
        return bool(self.trims(v, tilt_deg))

    # ------------------------------------------------------------ 走廊边界
    def tilt_bounds(self, v: float, step_deg: float = 0.5) -> tuple[float, float] | None:
        """``v`` 下可行的倾转角区间 ``(δ_min, δ_max)``；完全不可行返回 ``None``。"""
        key = round(v, 3)
        if key in self._bounds_cache:
            return self._bounds_cache[key]
        lo = hi = None
        n = int(round(90.0 / step_deg))
        for i in range(n + 1):
            tilt = i * step_deg
            if self.feasible(v, tilt):
                lo = tilt if lo is None else lo
                hi = tilt
        res = None if lo is None else (lo, hi)
        self._bounds_cache[key] = res
        return res

    def build_table(self, v_max: float = 32.0, dv: float = 0.25) -> None:
        """预计算走廊边界表，供控制回路每步查询（精确扫描太慢，不适合实时仿真）。"""
        vs, los, his = [], [], []
        n = int(round(v_max / dv))
        for i in range(n + 1):
            v = i * dv
            b = self.tilt_bounds(v)
            if b is None:
                break
            vs.append(v)
            los.append(b[0])
            his.append(b[1])
        self._table = (vs, los, his)

    def bounds(self, v: float, step_deg: float = 0.5) -> tuple[float, float] | None:
        """走廊边界查询：有预计算表就线性插值，否则精确扫描。"""
        if self._table is None:
            return self.tilt_bounds(v, step_deg)
        vs, los, his = self._table
        if not vs:
            return None
        if v <= vs[0]:
            return los[0], his[0]
        if v >= vs[-1]:
            # 超出表范围：最后一点仍可行则外推上限，否则视为不可行
            if v > vs[-1] * 1.02:
                return None
            return los[-1], his[-1]
        i = int(v / (vs[1] - vs[0]))
        i = min(max(i, 0), len(vs) - 2)
        f = (v - vs[i]) / (vs[i + 1] - vs[i])
        return los[i] + f * (los[i + 1] - los[i]), his[i] + f * (his[i + 1] - his[i])

    def margin_deg(self, v: float, tilt_deg: float, step_deg: float = 0.5) -> float:
        """倾转角到最近走廊边界的距离（度）。正=在走廊内，负=已越界。

        这是**离线分析**用的口径（走完整扫描，慢）；控制回路请用
        :meth:`lift_margin_deg`，它只算解析解，可以每步调用。
        """
        b = self.tilt_bounds(v, step_deg)
        if b is None:
            return -90.0
        lo, hi = b
        if tilt_deg < lo:
            return tilt_deg - lo
        if tilt_deg > hi:
            return hi - tilt_deg
        return min(tilt_deg - lo, hi - tilt_deg)

    def load_tilt_limit(self, v: float) -> float:
        """解析的升力侧下边界 ``δ_min(V)``（度）。

        只用垂直方向平衡推导：``sin δ ≥ (W - L_max(V)) / T_max(V)``。
        不走离散扫描，因此连续、无"空洞"，可以直接放在控制回路里每步算。

        与 :meth:`tilt_bounds` 的差别：后者还额外要求"水平推力压得过阻力"
        （定速平飞），在推重比临界点会出现无解的空洞；安全边界只关心托举，
        所以这里不含那条约束。
        """
        spec, rho = self.spec, self.rho
        l_max = spec.wing.max_lift(v, rho)
        need = spec.weight - l_max
        if need <= 0.0:
            return 0.0
        t_max = self.thrust_max(v)
        if t_max <= 1e-6:
            return 90.0
        return math.degrees(math.asin(min(1.0, need / t_max)))

    def lift_margin_deg(self, v: float, tilt_deg: float) -> float:
        """升力裕度：``δ - δ_min(V)``。小于 0 表示旋翼托不住全重，会掉高度。

        这才是**安全**边界。走廊的上边界 ``δ_max`` 限制的是"还能不能维持这个速度"，
        它不影响托举能力——减速时把倾转角往上拉是合法的，不该被它卡住。
        """
        d_min = self.load_tilt_limit(v)
        if d_min <= 0.5:
            # 机翼已能独担全重：垂直方向的旋翼约束不再起作用，裕度不适用
            return 90.0
        return tilt_deg - d_min

    def clamp_tilt(self, v: float, tilt_deg: float, margin: float = 3.0,
                   both_sides: bool = False) -> float:
        """把倾转角拉回走廊，并留 ``margin`` 度余量。

        默认只守**下边界**（升力侧）；``both_sides=True`` 时也限制上边界，
        用于校核"这条调度曲线能否维持速度"，不要用在控制回路里。
        """
        lo = self.load_tilt_limit(v)
        if both_sides:
            b = self.bounds(v)
            if b is None:
                return tilt_deg
            hi = b[1]
            lo2, hi2 = lo + margin, hi - margin
            if lo2 > hi2:
                lo2 = hi2 = 0.5 * (lo + hi)
            return min(max(tilt_deg, lo2), hi2)
        if tilt_deg >= lo:
            return tilt_deg
        return lo + margin

    def min_power_tilt(self, v: float, step_deg: float = 1.0) -> float | None:
        """准定常过渡的能耗最优参考倾转角。"""
        best: tuple[float, float] | None = None
        n = int(round(90.0 / step_deg))
        for i in range(n + 1):
            sol = self.level_flight(v, i * step_deg)
            if sol is None:
                continue
            if best is None or sol.power < best[0]:
                best = (sol.power, i * step_deg)
        return None if best is None else best[1]

    def corridor_polygon(self, v_max: float | None = None, dv: float = 0.25):
        """用于画图的走廊边界：``[(v, δ_min, δ_max), ...]``。"""
        v_max = v_max or self.max_level_speed()
        out = []
        v = 0.0
        while v <= v_max + 1e-9:
            b = self.tilt_bounds(v)
            if b is not None:
                out.append((v, b[0], b[1]))
            v += dv
        return out

    # ------------------------------------------------------------ 特征速度
    def max_level_speed(self, v_hi: float = 60.0) -> float:
        """任倾转角下都无法平飞的速度上限（推力线穿越阻力线处）。"""
        if self.tilt_bounds(v_hi) is not None:
            return v_hi
        lo, hi = 0.0, v_hi
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if self.tilt_bounds(mid) is not None:
                lo = mid
            else:
                hi = mid
        return lo

    @property
    def stall_speed(self) -> float:
        return self.spec.stall_speed(self.alt_m)

    @property
    def hover_speed_limit(self) -> float:
        return self.spec.hover_speed_limit(self.alt_m)

    @property
    def has_dead_band(self) -> bool:
        """是否存在"既不能悬停、也不能纯机翼平飞"的速度带——有它才需要过渡走廊。"""
        return self.hover_speed_limit < self.stall_speed

    def summary(self) -> dict[str, float | bool]:
        return {
            "stall_speed": round(self.stall_speed, 3),
            "hover_speed_limit": round(self.hover_speed_limit, 3),
            "max_level_speed": round(self.max_level_speed(), 3),
            "min_drag_speed": round(self.spec.min_drag_speed(self.alt_m), 3),
            "thrust_to_weight": round(self.spec.thrust_to_weight, 3),
            "has_dead_band": self.has_dead_band,
        }
