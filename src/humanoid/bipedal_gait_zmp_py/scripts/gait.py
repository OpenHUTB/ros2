"""周期解、步态仿真、开环不稳定性和增益扫描。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .dcm import clip_zmp, dcm, dcm_reference, stability_margin, zmp_command
from .footstep import FootstepPlan
from .lipm import OMEGA, lipm_step


@dataclass
class PeriodicSolution:
    """从第二只脚落地开始的单周期仿射映射周期解。"""

    t_start: float
    period: float
    displacement: float
    x0: float
    v0: float
    A: np.ndarray
    b: np.ndarray
    end_state: np.ndarray
    residual: float

    @property
    def trace(self) -> float:
        return float(np.trace(self.A))


@dataclass
class GaitResult:
    """一次闭环/开环仿真的完整时间序列与指标。"""

    gain: float
    dt: float
    time: np.ndarray
    x: np.ndarray
    v: np.ndarray
    zmp: np.ndarray
    zmp_ref: np.ndarray
    zmp_cmd: np.ndarray
    support_left: np.ndarray
    support_right: np.ndarray
    margin: np.ndarray
    dcm: np.ndarray
    dcm_ref: np.ndarray
    dcm_error: np.ndarray
    saturated: np.ndarray
    phase: np.ndarray
    travel: float
    avg_speed: float
    saturation_ratio: float
    mean_margin: float
    min_margin: float
    dcm_max_error: float
    final_velocity: float
    stable: bool
    extra: dict = field(default_factory=dict)

    def downsample(self, stride: int) -> "GaitResult":
        if stride <= 1:
            return self
        kw = {name: getattr(self, name) for name in self.__dataclass_fields__}
        for name in (
            "time",
            "x",
            "v",
            "zmp",
            "zmp_ref",
            "zmp_cmd",
            "support_left",
            "support_right",
            "margin",
            "dcm",
            "dcm_ref",
            "dcm_error",
            "saturated",
            "phase",
        ):
            kw[name] = getattr(self, name)[::stride]
        return GaitResult(**kw)


@dataclass
class InstabilityResult:
    """开环周期解扰动后的发散记录。"""

    cycle_errors: np.ndarray
    ratios: np.ndarray
    predicted_ratio: float
    final_x: float
    final_v: float


def _integrate_nominal(
    plan: FootstepPlan,
    x0: float,
    v0: float,
    t0: float,
    duration: float,
    dt: float,
) -> tuple[float, float]:
    """使用规划 ZMP 参考积分，不施加反馈。"""
    n = int(round(duration / dt))
    x, v = float(x0), float(v0)
    for k in range(n):
        t = t0 + k * dt
        p_ref = plan.support_state(t).zmp_ref
        x, v = lipm_step(x, v, p_ref, dt)
    return x, v


def compute_periodic_solution(plan: FootstepPlan, dt: float = 5e-4) -> PeriodicSolution:
    """用三个初值构造仿射映射，并解出周期解。

    周期起点取第二只脚落地时刻（t = step_time），此时步态已经完成起步，
    一个周期包含左右各一步，质心前进 2 * step_length。
    """
    t_start = plan.step_time
    period = plan.cycle_time
    displacement = 2.0 * plan.step_length

    b = np.array(_integrate_nominal(plan, 0.0, 0.0, t_start, period, dt))
    col1 = (
        np.array(_integrate_nominal(plan, 1.0, 0.0, t_start, period, dt)) - b
    )
    col2 = (
        np.array(_integrate_nominal(plan, 0.0, 1.0, t_start, period, dt)) - b
    )
    A = np.column_stack([col1, col2])

    matrix = np.array(
        [
            [A[0, 0] - 1.0, A[0, 1]],
            [A[1, 0], A[1, 1] - 1.0],
        ]
    )
    rhs = np.array([displacement - b[0], -b[1]])
    x0, v0 = np.linalg.solve(matrix, rhs)
    end_state = A @ np.array([x0, v0]) + b
    residual = float(
        max(
            abs(end_state[0] - (x0 + displacement)),
            abs(end_state[1] - v0),
        )
    )
    return PeriodicSolution(
        t_start=t_start,
        period=period,
        displacement=displacement,
        x0=float(x0),
        v0=float(v0),
        A=A,
        b=b,
        end_state=end_state,
        residual=residual,
    )


def simulate_gait(
    plan: FootstepPlan,
    gain: float,
    dt: float = 1e-3,
    initial_state: tuple[float, float] = (0.0, 0.0),
    duration: float | None = None,
    t_start: float = 0.0,
    v_avg: float | None = None,
) -> GaitResult:
    """按 DCM 反馈律仿真完整步态。

    p_cmd = p_ref + K (xi - xi_ref)，xi_ref = p_ref + v_avg / omega，
    随后把 p 限幅到支撑区间。
    """
    if duration is None:
        duration = plan.duration
    if v_avg is None:
        v_avg = plan.step_length / plan.step_time
    n = int(round(duration / dt))
    if n <= 0:
        raise ValueError("duration 必须至少包含一个积分步")

    x, v = map(float, initial_state)
    times = np.empty(n + 1)
    xs = np.empty(n + 1)
    vs = np.empty(n + 1)
    zmps = np.empty(n + 1)
    refs = np.empty(n + 1)
    cmds = np.empty(n + 1)
    los = np.empty(n + 1)
    his = np.empty(n + 1)
    margins = np.empty(n + 1)
    dcms = np.empty(n + 1)
    dcm_refs = np.empty(n + 1)
    errors = np.empty(n + 1)
    sat = np.zeros(n + 1, dtype=bool)
    phases = np.empty(n + 1, dtype=object)

    for k in range(n + 1):
        t = t_start + k * dt
        state = plan.support_state(t)
        xi = dcm(x, v)
        xi_ref = dcm_reference(state.zmp_ref, v_avg)
        p_cmd = zmp_command(xi, xi_ref, state.zmp_ref, gain)
        p = clip_zmp(p_cmd, state.left, state.right)
        margin = stability_margin(p, state.left, state.right)

        times[k] = t
        xs[k] = x
        vs[k] = v
        zmps[k] = p
        refs[k] = state.zmp_ref
        cmds[k] = p_cmd
        los[k] = state.left
        his[k] = state.right
        margins[k] = margin
        dcms[k] = xi
        dcm_refs[k] = xi_ref
        errors[k] = abs(xi - xi_ref)
        sat[k] = abs(p - p_cmd) > 1e-12
        phases[k] = state.phase

        if k == n:
            break
        x, v = lipm_step(x, v, p, dt)

    travel = float(xs[-1] - xs[0])
    avg_speed = travel / (n * dt)
    saturated_ratio = float(np.mean(sat))
    dcm_max = float(np.max(errors))
    finite = bool(
        np.all(np.isfinite(times))
        and np.all(np.isfinite(xs))
        and np.all(np.isfinite(vs))
        and np.all(np.isfinite(zmps))
        and np.all(np.isfinite(dcms))
        and np.isfinite(dcm_max)
    )
    # “走起来”的判据：有限、行程在合理范围、末速度不发散。
    stable = finite and 2.0 < travel < 5.0 and abs(float(vs[-1])) < 3.0

    return GaitResult(
        gain=float(gain),
        dt=float(dt),
        time=times,
        x=xs,
        v=vs,
        zmp=zmps,
        zmp_ref=refs,
        zmp_cmd=cmds,
        support_left=los,
        support_right=his,
        margin=margins,
        dcm=dcms,
        dcm_ref=dcm_refs,
        dcm_error=errors,
        saturated=sat,
        phase=phases,
        travel=travel,
        avg_speed=float(avg_speed),
        saturation_ratio=saturated_ratio,
        mean_margin=float(np.mean(margins)),
        min_margin=float(np.min(margins)),
        dcm_max_error=dcm_max,
        final_velocity=float(vs[-1]),
        stable=stable,
        extra={"v_avg": float(v_avg)},
    )


def run_open_loop_instability(
    plan: FootstepPlan,
    periodic: PeriodicSolution,
    cycles: int = 5,
    dt: float = 1e-3,
    perturbation: float = 1e-4,
) -> InstabilityResult:
    """给周期解一个很小扰动，记录每个周期的速度偏差。

    理论上一周期放大约 exp(omega * T_cycle)。T_cycle=1 s 时约为 33.17。
    """
    x = periodic.x0 + perturbation
    v = periodic.v0
    errors: list[float] = []
    n = int(round(periodic.period / dt))
    for cycle in range(cycles):
        t0 = periodic.t_start + cycle * periodic.period
        for k in range(n):
            p_ref = plan.support_state(t0 + k * dt).zmp_ref
            x, v = lipm_step(x, v, p_ref, dt)
        errors.append(v - periodic.v0)

    error_array = np.asarray(errors, dtype=float)
    ratios = error_array[1:] / error_array[:-1]
    return InstabilityResult(
        cycle_errors=error_array,
        ratios=ratios,
        predicted_ratio=float(math.exp(OMEGA * periodic.period)),
        final_x=float(x),
        final_v=float(v),
    )


def sweep_gains(
    plan: FootstepPlan,
    gains: list[float] | tuple[float, ...],
    dt: float = 1e-3,
) -> list[GaitResult]:
    """对给定增益逐一仿真。"""
    return [simulate_gait(plan, float(g), dt=dt) for g in gains]


def sweep_support_margin(
    margins: list[float] | tuple[float, ...] = (0.0, 0.01, 0.02, 0.03, 0.05, 0.08),
    gain: float = 1.0,
    dt: float = 1e-3,
    **plan_kwargs,
) -> list[dict[str, float | bool]]:
    """扫描“踝部边界缓冲”对同一增益是否还能走起来的影响。

    足底几何半长固定 0.09 m，只改变允许 ZMP 贴边到多远（有效半宽 =
    half_length + margin）。这是本模块最敏感的一个建模假设：缓冲为 0 时
    单脚支撑末段 ZMP 长期贴边，K=1 会发散。
    """
    rows: list[dict[str, float | bool]] = []
    for margin in margins:
        plan = FootstepPlan(
            half_length=plan_kwargs.get("half_length", 0.09),
            zmp_edge_margin=float(margin),
            n_steps=plan_kwargs.get("n_steps", 12),
            step_length=plan_kwargs.get("step_length", 0.35),
            step_time=plan_kwargs.get("step_time", 0.5),
        )
        result = simulate_gait(plan, gain, dt=dt)
        rows.append(
            {
                "zmp_edge_margin_m": float(margin),
                "effective_half_width_m": float(plan.support_half_width),
                "travel_m": float(result.travel),
                "avg_speed_mps": float(result.avg_speed),
                "saturation_ratio": float(result.saturation_ratio),
                "min_margin_m": float(result.min_margin),
                "stable": bool(result.stable),
            }
        )
    return rows


def result_to_row(result: GaitResult) -> dict[str, float | bool]:
    return {
        "gain": result.gain,
        "travel_m": result.travel,
        "avg_speed_mps": result.avg_speed,
        "saturation_ratio": result.saturation_ratio,
        "mean_margin_m": result.mean_margin,
        "min_margin_m": result.min_margin,
        "dcm_max_error_m": result.dcm_max_error,
        "final_velocity_mps": result.final_velocity,
        "stable": result.stable,
    }
