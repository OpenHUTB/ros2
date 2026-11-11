"""闭环仿真循环：控制器 -> 激活动力学 -> 刚体动力学。"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from .controller import ControlCommand, PDController
from .model import KneeModel, KneeState


@dataclass
class SimulationResult:
    """一次仿真的完整时间序列（角度类量均为 deg）。"""

    t: np.ndarray
    theta_deg: np.ndarray
    omega_rad_s: np.ndarray
    reference_deg: np.ndarray
    flex: np.ndarray
    ext: np.ndarray
    tau_pd: np.ndarray
    error_deg: np.ndarray
    valid: np.ndarray
    invalid_count: int
    upper_limit_count: int
    lower_limit_count: int
    saturated_count: int

    @property
    def error_actual_deg(self) -> np.ndarray:
        return self.theta_deg - self.reference_deg


def simulate(
    model: KneeModel,
    controller: PDController,
    reference_fn: Callable[[float], float],
    t: np.ndarray,
    initial_theta_deg: float,
    dt: float,
) -> SimulationResult:
    """从 initial_theta_deg 出发跑完给定时间向量。

    reference_fn 接受秒并返回目标角（deg）。控制器内部用 rad。
    """
    if dt <= 0:
        raise ValueError("dt 必须 > 0")
    if t.ndim != 1 or t.size == 0:
        raise ValueError("t 必须是非空一维数组")
    if not np.all(np.isfinite(t)):
        raise ValueError("t 不能包含 NaN/Inf")

    n = t.size
    theta = np.empty(n, dtype=float)
    omega = np.empty(n, dtype=float)
    reference = np.empty(n, dtype=float)
    flex = np.empty(n, dtype=float)
    ext = np.empty(n, dtype=float)
    tau_pd = np.empty(n, dtype=float)
    error = np.empty(n, dtype=float)
    valid = np.empty(n, dtype=bool)

    state = KneeState(
        theta=math.radians(initial_theta_deg),
        omega=0.0,
        flex=0.0,
        ext=0.0,
    )
    invalid_count = 0
    upper_count = 0
    lower_count = 0
    saturated_count = 0

    for i in range(n):
        ref_deg = float(reference_fn(float(t[i])))
        if not math.isfinite(ref_deg):
            raise ValueError("参考轨迹出现 NaN/Inf")
        command: ControlCommand = controller.compute(
            state.theta, state.omega, math.radians(ref_deg)
        )

        theta[i] = math.degrees(state.theta)
        omega[i] = state.omega
        reference[i] = ref_deg
        flex[i] = state.flex
        ext[i] = state.ext
        tau_pd[i] = command.tau_pd
        error[i] = math.degrees(state.theta) - ref_deg
        valid[i] = command.valid

        if not command.valid:
            invalid_count += 1
        if command.reason.endswith("_saturated"):
            saturated_count += 1

        # 用当前控制量向下一步推进；最后一步仍执行，保证状态数组完整。
        state = model.step(state, command.flex, command.ext, dt)

        if state.theta >= model.angle_max - 1e-9:
            upper_count += 1
        if state.theta <= model.angle_min + 1e-9:
            lower_count += 1

    return SimulationResult(
        t=np.asarray(t, dtype=float),
        theta_deg=theta,
        omega_rad_s=omega,
        reference_deg=reference,
        flex=flex,
        ext=ext,
        tau_pd=tau_pd,
        error_deg=error,
        valid=valid,
        invalid_count=invalid_count,
        upper_limit_count=upper_count,
        lower_limit_count=lower_count,
        saturated_count=saturated_count,
    )
