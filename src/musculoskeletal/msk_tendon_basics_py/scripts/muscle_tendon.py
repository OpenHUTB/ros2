"""肌肉-肌腱串联单元（Hill 肌纤维 + 线性弹性肌腱）。

本模块只依赖标准库与 numpy，用于复现 OpenHUTB 文档中的
“肌肉-肌腱串联弹性实验”。所有长度单位均为 m，力为 N，时间为 s。

符号约定
--------
``l_m``      肌纤维长度
``l_t``      肌腱长度（串联，``L_mtu = l_m + l_t``）
``l_n``      归一化肌纤维长度 ``l_m / l_opt``
``v_n``      归一化肌纤维速度 ``v_m / v_max``，**缩短为正**
``F_m``      肌纤维力
``F_t``      肌腱力（只能拉，不能推）
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MuscleTendonParams:
    """肌肉-肌腱单元参数（数值取自实验报告）。"""

    F_max: float = 120.0       # 最大等长收缩力 [N]
    l_opt: float = 0.10        # 最优肌纤维长度 [m]
    v_max: float = 1.2         # 最大缩短速度 [m/s]
    l_slack: float = 0.05      # 肌腱松弛长度 [m]
    tendon_strain: float = 0.08  # F_max 对应的肌腱应变 [-]
    width: float = 0.45        # 力-长度曲线宽度（归一化）[-]
    tau_act: float = 0.01      # 激活时间常数 [s]
    tau_deact: float = 0.04    # 去激活时间常数 [s]

    # 关节/负载参数也集中放在这里，便于联合实验复用。
    moment_arm: float = 0.03       # 关节力臂 [m]
    inertia: float = 0.05          # 关节转动惯量 [kg m^2]
    damping: float = 0.20          # 关节阻尼 [N m s/rad]
    rest_mtu: float = 0.135        # 关节中立位 MTU 长度 [m]
    load_stiffness: float = 6.0    # 负载弹簧刚度 [N m/rad]

    def __post_init__(self) -> None:
        for name in (
            "F_max",
            "l_opt",
            "v_max",
            "l_slack",
            "tendon_strain",
            "width",
            "tau_act",
            "tau_deact",
            "moment_arm",
            "inertia",
            "damping",
            "rest_mtu",
            "load_stiffness",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} 必须为有限正数，收到 {value!r}")

    @property
    def k_t(self) -> float:
        """由 8% 应变定义得到的线性肌腱刚度 [N/m]。

        ``k_t * (tendon_strain * l_slack) = F_max``。
        """

        return self.F_max / (self.tendon_strain * self.l_slack)


@dataclass(frozen=True)
class Equilibrium:
    """给定 MTU 长度与激活度时的串联力平衡结果。"""

    l_mtu: float
    activation: float
    velocity_n: float
    l_m: float
    l_t: float
    f_m: float
    f_t: float
    tendon_stiffness: float
    fiber_length_norm: float
    tendon_strain: float
    tendon_energy: float
    residual: float
    iterations: int
    converged: bool

    @property
    def force(self) -> float:
        """串联单元传力；平衡时 ``F_t == F_m``。"""

        return self.f_t

    @property
    def tendon_strain_percent(self) -> float:
        return 100.0 * self.tendon_strain


def activation_step(current: float, command: float, dt: float,
                    tau_act: float = 0.01, tau_deact: float = 0.04) -> float:
    """一阶激活动力学：上升用 ``tau_act``，下降用 ``tau_deact``。"""

    if dt < 0.0:
        raise ValueError("dt 不能为负")
    tau = tau_act if command >= current else tau_deact
    alpha = 1.0 - math.exp(-dt / tau)
    return current + alpha * (command - current)


class MuscleTendon:
    """串联肌肉-肌腱单元。

    肌腱为线性弹性且只能承受拉力::

        F_t = k_t * max(0, l_t - l_slack)

    肌纤维采用 Hill 型力-长度-速度关系::

        F_m = F_max * [ a * f_l(l_n) * f_v(v_n) + f_p(l_n) ]
    """

    def __init__(self, params: MuscleTendonParams | None = None,
                 tendon_stiffness: float | None = None) -> None:
        self.params = params if params is not None else MuscleTendonParams()
        self.tendon_stiffness = (
            self.params.k_t if tendon_stiffness is None else float(tendon_stiffness)
        )
        if math.isnan(self.tendon_stiffness) or self.tendon_stiffness <= 0.0:
            raise ValueError("tendon_stiffness 必须为正数或 +inf")

    # ------------------------------------------------------------------
    # 本构曲线
    # ------------------------------------------------------------------
    def force_length(self, l_n: float) -> float:
        """主动力-长度曲线，峰值在 ``l_n = 1`` 处。"""

        return math.exp(-((l_n - 1.0) / self.params.width) ** 2)

    def passive_force_length(self, l_n: float) -> float:
        """被动力-长度曲线：``l_n <= 1`` 时为 0，超过最优长度后二次增长。"""

        if l_n <= 1.0:
            return 0.0
        return ((l_n - 1.0) / self.params.width) ** 2

    def force_velocity(self, v_n: float) -> float:
        """力-速度曲线，缩短为正。

        缩短支使用经典 Hill 双曲线 ``(1-v)/(1+v/0.25)``；
        拉长支取线性增强 ``1 + 0.4 * min(-v, 1)``，保证连续且为正。
        """

        if v_n >= 1.0:
            return 0.0
        if v_n >= 0.0:
            return (1.0 - v_n) / (1.0 + v_n / 0.25)
        return 1.0 + 0.4 * min(-v_n, 1.0)

    def fiber_force(self, l_m: float, activation: float,
                    velocity_n: float = 0.0) -> float:
        """肌纤维总力（主动 + 被动）。"""

        l_n = l_m / self.params.l_opt
        active = activation * self.force_length(l_n) * self.force_velocity(velocity_n)
        passive = self.passive_force_length(l_n)
        return self.params.F_max * (active + passive)

    def tendon_force(self, l_t: float, tendon_stiffness: float | None = None) -> float:
        """肌腱力；``l_t <= l_slack`` 时为零。"""

        k = self.tendon_stiffness if tendon_stiffness is None else float(tendon_stiffness)
        return k * max(0.0, l_t - self.params.l_slack)

    # ------------------------------------------------------------------
    # 串联平衡
    # ------------------------------------------------------------------
    def _residual(self, l_m: float, l_mtu: float, activation: float,
                  velocity_n: float, stiffness: float) -> float:
        l_t = l_mtu - l_m
        f_t = stiffness * max(0.0, l_t - self.params.l_slack)
        f_m = self.fiber_force(l_m, activation, velocity_n)
        return f_t - f_m

    def solve_fiber_length(self, l_mtu: float, activation: float,
                           velocity_n: float = 0.0,
                           tendon_stiffness: float | None = None,
                           tol: float = 1e-12,
                           max_iter: int = 200) -> float:
        """二分法求解满足 ``F_t(l_m) = F_m(l_m)`` 的肌纤维长度。

        残差 ``F_t - F_m`` 在区间 ``[0, l_mtu]`` 上从正变负，二分法单调收敛。
        刚性肌腱（``tendon_stiffness=inf``）直接取极限解
        ``l_m = max(0, l_mtu - l_slack)``。
        """

        if not math.isfinite(l_mtu) or l_mtu < 0.0:
            raise ValueError(f"l_mtu 必须为非负有限数，收到 {l_mtu!r}")
        if not math.isfinite(activation) or activation < 0.0:
            raise ValueError(f"activation 必须为非负有限数，收到 {activation!r}")
        if tol <= 0.0 or max_iter <= 0:
            raise ValueError("tol 必须为正且 max_iter 必须为正")

        k = self.tendon_stiffness if tendon_stiffness is None else float(tendon_stiffness)
        if math.isnan(k) or k <= 0.0:
            raise ValueError("tendon_stiffness 必须为正数或 +inf")

        # 刚性极限：肌腱长度固定为松弛长度，MTU 的全部长度变化都由肌纤维承担。
        if math.isinf(k):
            return max(0.0, l_mtu - self.params.l_slack)

        # 无激活或肌腱始终松弛时，串联力为零；取肌腱恰好松弛的构型。
        if activation <= 0.0 or l_mtu <= self.params.l_slack:
            return max(0.0, l_mtu - self.params.l_slack)

        lo = 0.0
        hi = l_mtu
        f_lo = self._residual(lo, l_mtu, activation, velocity_n, k)
        f_hi = self._residual(hi, l_mtu, activation, velocity_n, k)

        if f_lo <= 0.0:
            return lo
        if f_hi >= 0.0:
            return hi

        for _ in range(max_iter):
            mid = 0.5 * (lo + hi)
            f_mid = self._residual(mid, l_mtu, activation, velocity_n, k)
            if f_mid > 0.0:
                lo = mid
            else:
                hi = mid
            # 正常刚度下达到 1e-12 即可；刚度极高时继续二分，
            # 保证最终力残差仍小于 1e-6（浮点分辨率允许时）。
            if hi - lo <= tol and abs(f_mid) <= 1e-6:
                break

        return 0.5 * (lo + hi)

    def equilibrium(self, l_mtu: float, activation: float,
                    velocity_n: float = 0.0,
                    tendon_stiffness: float | None = None,
                    tol: float = 1e-12,
                    max_iter: int = 200) -> Equilibrium:
        """求串联平衡并回代肌腱长度、应变、储能与残差。"""

        if not math.isfinite(l_mtu) or l_mtu < 0.0:
            raise ValueError(f"l_mtu 必须为非负有限数，收到 {l_mtu!r}")
        if not math.isfinite(activation) or activation < 0.0:
            raise ValueError(f"activation 必须为非负有限数，收到 {activation!r}")

        k = self.tendon_stiffness if tendon_stiffness is None else float(tendon_stiffness)
        if math.isnan(k) or k <= 0.0:
            raise ValueError("tendon_stiffness 必须为正数或 +inf")

        if math.isinf(k):
            l_m = max(0.0, l_mtu - self.params.l_slack)
            l_t = l_mtu - l_m
            f_m = self.fiber_force(l_m, activation, velocity_n)
            f_t = f_m
            energy = 0.0
            strain = 0.0
            residual = 0.0
            iterations = 0
            converged = True
        elif l_mtu <= self.params.l_slack:
            # 肌腱全程松弛，串联单元不能传力。
            l_m = l_mtu
            l_t = 0.0
            f_m = 0.0
            f_t = 0.0
            energy = 0.0
            strain = 0.0
            residual = 0.0
            iterations = 0
            converged = True
        else:
            l_m = self.solve_fiber_length(
                l_mtu, activation, velocity_n, k, tol=tol, max_iter=max_iter
            )
            l_t = l_mtu - l_m
            f_t = k * max(0.0, l_t - self.params.l_slack)
            f_m = self.fiber_force(l_m, activation, velocity_n)
            energy = f_t * f_t / (2.0 * k)
            strain = max(0.0, l_t - self.params.l_slack) / self.params.l_slack
            residual = f_t - f_m
            iterations = max_iter
            # 用二分区间长度无法从外部获得，这里给出保守的收敛判断。
            converged = math.isfinite(l_m) and abs(residual) < 1e-6

        return Equilibrium(
            l_mtu=l_mtu,
            activation=activation,
            velocity_n=velocity_n,
            l_m=l_m,
            l_t=l_t,
            f_m=f_m,
            f_t=f_t,
            tendon_stiffness=k,
            fiber_length_norm=l_m / self.params.l_opt,
            tendon_strain=strain,
            tendon_energy=energy,
            residual=residual,
            iterations=iterations,
            converged=converged,
        )

    # ------------------------------------------------------------------
    # 批量工具
    # ------------------------------------------------------------------
    def sweep_activation(self, l_mtu: float, activations: np.ndarray,
                         tendon_stiffness: float | None = None) -> list[Equilibrium]:
        return [
            self.equilibrium(l_mtu, float(a), tendon_stiffness=tendon_stiffness)
            for a in np.asarray(activations, dtype=float)
        ]


def rigid_force_limit(params: MuscleTendonParams | None = None,
                      activation: float = 1.0,
                      l_mtu: float | None = None) -> float:
    """解析计算 ``k_t -> inf`` 时的传力（用于对照与测试）。"""

    p = params if params is not None else MuscleTendonParams()
    length = p.rest_mtu if l_mtu is None else l_mtu
    l_m = max(0.0, length - p.l_slack)
    l_n = l_m / p.l_opt
    return p.F_max * (activation * math.exp(-((l_n - 1.0) / p.width) ** 2)
                      + (0.0 if l_n <= 1.0 else ((l_n - 1.0) / p.width) ** 2))


# ----------------------------------------------------------------------
# 模块级便捷函数（便于脚本和外部测试直接调用）
# ----------------------------------------------------------------------
def solve_fiber_length(l_mtu: float, activation: float,
                       params: MuscleTendonParams | None = None,
                       tendon_stiffness: float | None = None,
                       velocity_n: float = 0.0,
                       tol: float = 1e-12,
                       max_iter: int = 200) -> float:
    """``MuscleTendon.solve_fiber_length`` 的模块级封装。"""

    return MuscleTendon(params, tendon_stiffness).solve_fiber_length(
        l_mtu, activation, velocity_n=velocity_n,
        tol=tol, max_iter=max_iter,
    )


def equilibrium(l_mtu: float, activation: float,
                params: MuscleTendonParams | None = None,
                tendon_stiffness: float | None = None,
                velocity_n: float = 0.0,
                tol: float = 1e-12,
                max_iter: int = 200) -> Equilibrium:
    """``MuscleTendon.equilibrium`` 的模块级封装。"""

    return MuscleTendon(params, tendon_stiffness).equilibrium(
        l_mtu, activation, velocity_n=velocity_n,
        tol=tol, max_iter=max_iter,
    )
