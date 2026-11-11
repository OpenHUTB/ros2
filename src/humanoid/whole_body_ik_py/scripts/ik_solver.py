"""逆运动学求解器：伪逆、阻尼最小二乘（DLS）与自适应阻尼。

迭代式（末端位姿误差 e = [Δp; Δω]）：

    Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e

λ = 0 时退化为最小范数伪逆；λ 随最小奇异值 σ_min 自适应，避免奇异位形附近
关节步长按 1/σ_min 爆掉。

零空间次级任务：

    Δq = J⁺ e + (I − J⁺ J) z

第二项不改变末端位姿，只用来做“远离关节限位”这类次级优化。

这是**局部**方法：收敛依赖初值，目标超出可达空间时返回 success=False。
默认初值取关节限位中点，避免全零（完全伸直）这种奇异位形。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .kinematics import SerialChain
from .linalg import (
    adaptive_damping,
    dls_step,
    joint_limit_gradient,
    nullspace_projector,
    orientation_error,
    pinv_step,
    singular_values,
)


@dataclass
class IKResult:
    q: np.ndarray
    iterations: int
    error: float
    success: bool
    method: str
    history: list[float] = field(default_factory=list)
    sigma_min: float = 0.0
    damping: float = 0.0
    q_initial: np.ndarray | None = None

    @property
    def converged(self) -> bool:
        return bool(self.success)


def pose_error(target_p, target_r, p, r) -> np.ndarray:
    """6 维位姿误差 [位置; 姿态旋转向量]。"""
    return np.concatenate(
        [
            np.asarray(target_p, dtype=float).reshape(3) - p,
            orientation_error(np.asarray(target_r, dtype=float), r),
        ]
    )


def _initial_guess(chain: SerialChain, q0) -> np.ndarray:
    if q0 is None:
        return chain.q_mid.copy()
    q = np.asarray(q0, dtype=float).reshape(-1)
    if q.size != chain.dof:
        raise ValueError(f"q0 需要 {chain.dof} 个分量，收到 {q.size} 个")
    return q.copy()


def solve_pose_ik(
    chain: SerialChain,
    target_p,
    target_r,
    q0=None,
    *,
    method: str = "dls",
    max_iter: int = 500,
    tol: float = 1e-6,
    step_max: float = 0.25,
    damping_threshold: float = 0.05,
    lambda_max: float = 0.2,
    escape_after: int = 15,
    escape_scale: float = 0.03,
    seed: int = 0,
    nullspace_gain: float = 0.0,
    record_history: bool = True,
) -> IKResult:
    """求解 6 维位姿逆解。

    参数
    ----
    method : "dls" 或 "pinv"
    escape_after : 连续多少步没有改进就做一次小扰动脱困；0 表示关闭
    nullspace_gain : >0 时把关节限位梯度投影到零空间，作为次级任务
    """
    if method not in ("dls", "pinv"):
        raise ValueError("method 必须是 'dls' 或 'pinv'")
    target_p = np.asarray(target_p, dtype=float).reshape(3)
    target_r = np.asarray(target_r, dtype=float).reshape(3, 3)
    q_initial = _initial_guess(chain, q0)
    q = np.clip(q_initial, chain.lower, chain.upper)

    rng = np.random.default_rng(seed)
    best = np.inf
    stall = 0
    history: list[float] = []
    sigma_min = 0.0
    damping = 0.0
    it = 0

    for it in range(max_iter + 1):
        fk = chain.forward(q)
        err = pose_error(target_p, target_r, fk.position, fk.rotation)
        norm = float(np.linalg.norm(err))
        history.append(norm)
        if norm < tol:
            return IKResult(q=q, iterations=it, error=norm, success=True,
                            method=method, history=history,
                            sigma_min=sigma_min, damping=damping,
                            q_initial=q_initial)

        jac = chain.jacobian(q)
        s = singular_values(jac)
        sigma_min = float(s[-1]) if s.size else 0.0

        if method == "pinv":
            damping = 0.0
            dq = pinv_step(jac, err)
        else:
            damping = adaptive_damping(sigma_min, threshold=damping_threshold,
                                       lambda_min=0.0, lambda_max=lambda_max)
            dq = dls_step(jac, err, damping)

        if nullspace_gain > 0.0:
            # 次级任务：把关节往区间内部推，投影到零空间后不改变末端位姿。
            projector = nullspace_projector(jac)
            direction = projector @ joint_limit_gradient(q, chain.lower, chain.upper)
            largest = float(np.max(np.abs(direction))) if direction.size else 0.0
            if largest > 1e-12:
                dq = dq + nullspace_gain * direction / largest
        step = float(np.linalg.norm(dq))
        if step > step_max:
            dq *= step_max / step
        q = np.clip(q + dq, chain.lower, chain.upper)

        if norm < best - 1e-12:
            best = norm
            stall = 0
        else:
            stall += 1
        if escape_after > 0 and stall >= escape_after:
            width = chain.upper - chain.lower
            q = np.clip(q + rng.normal(0.0, escape_scale, chain.dof) * width,
                        chain.lower, chain.upper)
            stall = 0

    fk = chain.forward(q)
    final = float(np.linalg.norm(pose_error(target_p, target_r, fk.position, fk.rotation)))
    return IKResult(q=q, iterations=it, error=final, success=False,
                    method=method, history=history, sigma_min=sigma_min,
                    damping=damping, q_initial=q_initial)


def solve_position_ik(
    chain: SerialChain,
    target_p,
    q0=None,
    *,
    rows=(0, 1, 2),
    max_iter: int = 300,
    tol: float = 1e-6,
    step_max: float = 0.2,
) -> IKResult:
    """只解位置（取雅可比的部分行）。2R 平面臂用 rows=(0, 1)。

    直接用 3 维位置雅可比会把所有位形判成奇异：z 方向永远没有速度。
    """
    rows = tuple(int(r) for r in rows)
    target_p = np.asarray(target_p, dtype=float).reshape(3)
    q = np.clip(_initial_guess(chain, q0), chain.lower, chain.upper)
    history: list[float] = []

    for it in range(max_iter + 1):
        p = chain.forward(q).position
        err = (target_p - p)[list(rows)]
        norm = float(np.linalg.norm(err))
        history.append(norm)
        if norm < tol:
            return IKResult(q=q, iterations=it, error=norm, success=True,
                            method="position", history=history)
        sub = chain.jacobian(q)[list(rows), :]
        dq = sub.T @ np.linalg.solve(sub @ sub.T + 1e-12 * np.eye(len(rows)), err)
        step = float(np.linalg.norm(dq))
        if step > step_max:
            dq *= step_max / step
        q = np.clip(q + dq, chain.lower, chain.upper)

    p = chain.forward(q).position
    final = float(np.linalg.norm((target_p - p)[list(rows)]))
    return IKResult(q=q, iterations=max_iter, error=final, success=False,
                    method="position", history=history)


def track_circle(
    chain: SerialChain,
    center,
    radius: float,
    n_points: int = 25,
    plane: str = "xz",
    *,
    method: str = "dls",
    tol: float = 1e-6,
    warm_start: bool = True,
    max_iter: int = 500,
) -> dict:
    """末端沿圆轨迹走一圈，每个点用上一点的解做初值。

    位姿参考由 kinematics.reference_circle_pose 给出（平面 2R 解析解构造的
    可达全位姿），因此成功与否只取决于 IK 数值行为，不是目标不可达。
    """
    from .kinematics import reference_circle_pose

    center = np.asarray(center, dtype=float).reshape(3)
    q = chain.q_mid.copy()
    errors: list[float] = []
    iterations: list[int] = []
    targets: list[np.ndarray] = []
    solutions: list[np.ndarray] = []
    success: list[bool] = []

    for k in range(n_points):
        angle = 2.0 * np.pi * k / n_points
        p, r, _ = reference_circle_pose(chain, center, radius, angle, plane)
        result = solve_pose_ik(
            chain, p, r, q if warm_start else None, method=method,
            tol=tol, max_iter=max_iter, seed=k,
            record_history=False,
        )
        q = result.q
        errors.append(result.error)
        iterations.append(result.iterations)
        targets.append(np.asarray(p, dtype=float))
        solutions.append(result.q.copy())
        success.append(result.success)

    return {
        "errors": np.asarray(errors),
        "iterations": np.asarray(iterations, dtype=int),
        "targets": np.asarray(targets),
        "solutions": np.asarray(solutions),
        "success": np.asarray(success, dtype=bool),
    }
