"""四轮纵向力分配：带附着椭圆约束的加权最小二乘。

要解的是：

.. math::

    \\min_{F_i} \\sum_i \\left(\\frac{F_i}{c_i}\\right)^2
    \\quad \\text{s.t.} \\quad \\sum_i F_i = F_{demand},\\;
    \\sum_i m_i F_i = M_z

`c_i` 是每个轮子的纵向力额度（附着椭圆 ∩ 电机能力），`m_i = ±t/2` 是力臂。
目标函数是"归一化轮胎负荷"，所以分配天然倾向`让四个轮子同样吃力`，
而不是把需求堆到某一个轮子上。

不加饱和约束时它等价于

.. math::

    F = W^{-1}A^{\\top}(A W^{-1} A^{\\top})^{-1} b,\\qquad W = \\operatorname{diag}(1/c_i^2)

只有 2×2 的矩阵要求逆，可以每步都算。碰到饱和就用**有效集**：把顶到边界的轮子固定住，
扣掉它已经提供的力与力矩，再对剩下的轮子重新解一次。最多 4 轮，迭代 4 次必收敛。
"""

from __future__ import annotations

from dataclasses import dataclass, field

__all__ = ["AllocResult", "allocate", "allocate_equal"]


@dataclass
class AllocResult:
    fx: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    achieved_total: float = 0.0
    achieved_moment: float = 0.0
    saturated: bool = False
    authoritative_wheels: int = 4

    @property
    def total_error(self) -> float:
        return self.achieved_total

    def as_tuple(self) -> tuple[float, ...]:
        return tuple(self.fx)


def _solve_unconstrained(demand: float, moment: float,
                         caps: list[float], arms: list[float],
                         free: list[int]) -> dict[int, float]:
    """在 ``free`` 指定的自由轮子上解等式约束最小二乘。"""
    # W = diag(1/cap²) → W⁻¹ = diag(cap²)；权重必须用 cap²，
    # 否则会变成"额度小的轮子分到更多力"，与目标函数正好相反。
    s0 = s1 = s2 = 0.0
    for i in free:
        w = caps[i] * caps[i]
        ci = arms[i]
        s0 += w
        s1 += w * ci
        s2 += w * ci * ci
    det = s0 * s2 - s1 * s1
    if abs(det) < 1e-12:
        # 退化（例如只剩同侧轮子）：只满足驱动力需求，力矩尽力而为
        out = {}
        total_w = sum(caps[i] ** 2 for i in free)
        for i in free:
            out[i] = demand * (caps[i] ** 2) / total_w
        return out
    lam = (s2 * demand - s1 * moment) / det
    mu = (-s1 * demand + s0 * moment) / det
    return {i: (lam + mu * arms[i]) * (caps[i] ** 2) for i in free}


def allocate(demand: float, moment: float, caps: list[float],
             arms: list[float], max_iter: int = 4) -> AllocResult:
    """把总驱动力 ``demand`` 与目标横摆力矩 ``moment`` 分配到四个轮子。

    参数
    ----
    demand : 总纵向力需求，N
    moment : 目标横摆力矩，N·m（逆时针为正）
    caps   : 每轮纵向力额度，N（附着椭圆与电机能力的较严者）
    arms   : 每轮力臂，m；左负右正
    """
    if len(caps) != 4 or len(arms) != 4:
        raise ValueError("必须是四轮")
    caps = [max(0.0, c) for c in caps]
    res = AllocResult()

    free = [i for i in range(4) if caps[i] > 1e-9]
    if not free:
        res.saturated = True
        res.authoritative_wheels = 0
        return res

    fixed: dict[int, float] = {}
    demand_left = demand
    moment_left = moment

    for _ in range(max_iter):
        if not free:
            break
        sol = _solve_unconstrained(demand_left, moment_left, caps, arms, free)
        overflow = [i for i in free if abs(sol[i]) > caps[i] + 1e-6]
        if not overflow:
            for i in free:
                res.fx[i] = sol[i]
            break
        # 把越界最多的那个钉在边界上，扣掉它的贡献后继续
        worst = max(overflow, key=lambda i: abs(sol[i]) - caps[i])
        bound = caps[worst] if sol[worst] > 0 else -caps[worst]
        fixed[worst] = bound
        res.fx[worst] = bound
        demand_left -= bound
        moment_left -= bound * arms[worst]
        free = [i for i in free if i != worst]
    else:
        sol = _solve_unconstrained(demand_left, moment_left, caps, arms, free) if free else {}
        for i, v in sol.items():
            res.fx[i] = v

    res.saturated = bool(fixed)
    res.authoritative_wheels = len(free)
    res.achieved_total = sum(res.fx)
    res.achieved_moment = sum(f * m for f, m in zip(res.fx, arms))
    return res


def allocate_equal(demand: float, caps: list[float]) -> AllocResult:
    """平均分配（开放式差速器 / 无扭矩矢量的对照组）。

    每个轮子拿同样的力，谁先撞到附着上限谁先饱和——正是这种"各凭本事"的分法，
    让分离路面制动时低附着侧先打滑，产生一个**没人管的横摆力矩**。
    这正是扭矩矢量要解决的问题，也是本模块对照组的意义。
    """
    n = len(caps)
    share = demand / n
    res = AllocResult()
    res.fx = [max(-c, min(c, share)) if c > 1e-9 else 0.0 for c in caps]
    res.saturated = any(abs(f) >= c - 1e-6 for f, c in zip(res.fx, caps))
    res.authoritative_wheels = sum(1 for c in caps if c > 1e-9)
    res.achieved_total = sum(res.fx)
    res.achieved_moment = 0.0
    return res
