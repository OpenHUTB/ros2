"""可操作性、奇异值、速度椭球与零空间维数。

    w = sqrt(det(J Jᵀ))

几何意义是末端速度椭球的体积；J 的奇异值就是椭球半轴。平面 2R 臂取 x、y
两行时有闭式解 w = |l1 l2 sin q1|。

对平面机构必须裁剪任务维度：3 维位置雅可比的 z 行恒为 0，会让所有位形都被
判成奇异。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import SerialChain
from .linalg import (
    adaptive_damping,
    condition_number,
    pseudoinverse,
    singular_values,
)


@dataclass(frozen=True)
class Measure:
    value: float
    singular_values: np.ndarray
    condition: float
    task_dim: int

    @property
    def sigma_min(self) -> float:
        return float(self.singular_values[-1]) if self.singular_values.size else 0.0

    @property
    def sigma_max(self) -> float:
        return float(self.singular_values[0]) if self.singular_values.size else 0.0

    @property
    def is_singular(self) -> bool:
        return self.sigma_min < 1e-6


def task_jacobian(chain: SerialChain, q, rows=None) -> np.ndarray:
    """按任务维度裁剪雅可比；rows=None 表示完整的 6 维位姿任务。"""
    jac = chain.jacobian(q)
    if rows is None:
        return jac
    return jac[list(int(r) for r in rows), :]


def manipulability(jac: np.ndarray) -> Measure:
    """Yoshikawa 可操作性指标。"""
    jac = np.asarray(jac, dtype=float)
    if jac.ndim != 2:
        raise ValueError("雅可比必须是二维矩阵")
    s = singular_values(jac)
    if jac.shape[0] > s.size:
        # m > n：svd 只给 n 个奇异值，其余 m-n 个恒为 0。
        s = np.concatenate([s, np.zeros(jac.shape[0] - s.size)])
    s = np.sort(s)[::-1]
    m, n = jac.shape
    if m <= n:
        value = float(np.sqrt(max(float(np.linalg.det(jac @ jac.T)), 0.0)))
    else:
        value = float(np.sqrt(max(float(np.linalg.det(jac.T @ jac)), 0.0)))
    condition = condition_number(jac)
    return Measure(value=value, singular_values=s, condition=condition, task_dim=m)


def velocity_ellipsoid(jac: np.ndarray, n_dirs: int = 121) -> dict:
    """2 维任务的末端速度椭球：v = J u，|u| = 1。"""
    jac = np.asarray(jac, dtype=float)
    if jac.shape[0] != 2:
        raise ValueError("椭球可视化只支持 2 维任务")
    u, s, vt = np.linalg.svd(jac, full_matrices=False)
    theta = np.linspace(0.0, 2.0 * np.pi, n_dirs)
    unit = np.vstack([np.cos(theta), np.sin(theta)])
    # 主轴方向取 J 的右奇异向量 vt 的行（每个奇异值对应一列特征方向）。
    major = vt[0].copy()
    minor = vt[1].copy() if vt.shape[0] > 1 else np.array([-major[1], major[0]])
    return {
        "semi_axes": s.copy(),
        "major_axis": major,
        "minor_axis": minor,
        "points": jac @ unit,
    }


def elbow_sweep(chain: SerialChain, elbow_values, l1: float, l2: float,
                base_angle: float = 0.0) -> dict:
    """2R 臂肘角扫描：数值可操作性 vs 闭式解。"""
    values = np.asarray(elbow_values, dtype=float)
    numeric = np.empty(values.size)
    closed = np.empty(values.size)
    cond = np.empty(values.size)
    for k, elbow in enumerate(values):
        q = np.array([base_angle, float(elbow)])
        measure = manipulability(task_jacobian(chain, q, rows=(0, 1)))
        numeric[k] = measure.value
        closed[k] = abs(l1 * l2 * np.sin(float(elbow)))
        cond[k] = measure.condition
    return {"elbow": values, "numeric": numeric, "closed_form": closed,
            "condition": cond}


def singularity_scan(chain: SerialChain, elbow_values,
                     radial_error: float = 0.01,
                     damping_threshold: float = 0.05,
                     lambda_max: float = 0.2) -> list[dict]:
    """沿 +x 方向给固定末端误差，比较伪逆与 DLS 的关节步长。"""
    rows = []
    direction = np.array([1.0, 0.0]) * float(radial_error)
    for elbow in np.asarray(elbow_values, dtype=float):
        q = np.array([0.0, float(elbow)])
        jac = task_jacobian(chain, q, rows=(0, 1))
        measure = manipulability(jac)
        step_pinv = float(np.linalg.norm(pseudoinverse(jac) @ direction))
        lam = adaptive_damping(measure.sigma_min, threshold=damping_threshold,
                               lambda_min=0.0, lambda_max=lambda_max)
        u, s, vt = np.linalg.svd(jac, full_matrices=False)
        denom = s**2 + lam**2
        factors = np.divide(s, denom, out=np.zeros_like(s), where=denom > 1e-18)
        step_dls = float(np.linalg.norm(vt.T @ (factors * (u.T @ direction))))
        rows.append(
            {
                "elbow": float(elbow),
                "sigma_min": measure.sigma_min,
                "step_pinv": step_pinv,
                "step_dls": step_dls,
                "adaptive_lambda": lam,
                "ratio": (step_dls / step_pinv) if step_pinv > 0 else 0.0,
            }
        )
    return rows


def nullspace_report(chain: SerialChain, q, seed: int = 0) -> dict:
    """零空间投影：|J (I − J⁺J) z| 应接近 0，而关节被实际调整。"""
    jac = chain.jacobian(q)
    projector = np.eye(chain.dof) - pseudoinverse(jac) @ jac
    rng = np.random.default_rng(seed)
    z = rng.normal(size=chain.dof)
    projected = projector @ z
    return {
        "rank": int(np.linalg.matrix_rank(jac)),
        "nullity": int(chain.dof - np.linalg.matrix_rank(jac)),
        "dq_norm": float(np.linalg.norm(projected)),
        "end_effector_motion": float(np.linalg.norm(jac @ projected)),
    }
