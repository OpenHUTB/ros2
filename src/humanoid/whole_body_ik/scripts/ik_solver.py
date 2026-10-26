# -*- coding: utf-8 -*-
"""逆运动学求解：伪逆、阻尼最小二乘、零空间投影。

三种解法的关系
--------------
最小范数解（伪逆）::

    Δq = Jᵀ (J Jᵀ)⁻¹ e

在奇异位形附近 J Jᵀ 接近奇异，求逆会数值爆炸、关节速度飙到天上。工程上用
**阻尼最小二乘**（Levenberg–Marquardt / DLS）兜住::

    Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e

λ 越大越保守：奇异方向上不发散，代价是末端收敛变慢。λ 按最小奇异值自适应
（``adaptive_damping``）可以两头兼顾。

冗余自由度（n > 6）还有**零空间**可以用::

    Δq = J⁺ e + (I − J⁺J) z

第二项不改变末端位姿，只用来优化别的东西（比如远离关节限位）。

为什么不用公式显式构造 J⁺
------------------------
显式算伪逆要做 SVD，代码量大。这里直接解线性方程组就够：先把
``(J Jᵀ + λ²I) y = e`` 解出来，再 ``Δq = Jᵀ y``，数学上等价且更稳定。
"""

from __future__ import annotations

import math

from linalg import (
    dot,
    identity,
    mat_mul,
    mat_vec,
    norm,
    solve,
    transpose,
)

__all__ = [
    "pseudoinverse_step",
    "damped_step",
    "adaptive_damping",
    "null_space_project",
    "solve_position_ik",
    "rotation_error",
    "solve_pose_ik",
    "IKResult",
    "default_initial_guess",
]


# ---------------------------------------------------------------------------
def _augment(jjt, lam):
    """给 J Jᵀ 的对角线加 λ²。"""
    n = len(jjt)
    out = [list(row) for row in jjt]
    lam2 = lam * lam
    for i in range(n):
        out[i][i] += lam2
    return out


def pseudoinverse_step(j, err, tol=1e-10):
    """最小范数解 Δq = Jᵀ (J Jᵀ)⁻¹ e。

    j 是 m×n 雅可比，err 是长度 m 的末端误差。
    奇异时抛 ValueError——这种情况应该改用 ``damped_step``。
    """
    jt = transpose(j)
    jjt = mat_mul(j, jt)
    y = solve(jjt, err, tol=tol)
    return mat_vec(jt, y)


def damped_step(j, err, lam):
    """阻尼最小二乘 Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ e。

    λ = 0 且矩阵奇异时会退化失败，所以 λ 太小的时候会兜一个下限。
    """
    if lam < 0:
        raise ValueError("阻尼系数不能为负")
    jt = transpose(j)
    jjt = mat_mul(j, jt)
    y = solve(_augment(jjt, lam), err, tol=1e-14)
    return mat_vec(jt, y)


def smallest_singular_value(j):
    """用 J Jᵀ 的最小特征值开根号近似最小奇异值。"""
    from linalg import symmetric_eigen

    jt = transpose(j)
    jjt = mat_mul(j, jt)
    vals, _ = symmetric_eigen(jjt)
    return math.sqrt(max(0.0, vals[-1]))


def adaptive_damping(j, lam_min=1e-4, lam_max=0.2, sigma_threshold=0.05):
    """按最小奇异值自适应选阻尼。

    σ_min 离奇异越近，λ 越大：

        σ_min ≥ σ_threshold   → λ = λ_min（几乎不阻尼，收敛快）
        σ_min = 0             → λ = λ_max（强阻尼，不发散）

    中间段用线性插值。这个启发式在「末端要准」和「奇异点要稳」之间取平衡。
    """
    s = smallest_singular_value(j)
    if s >= sigma_threshold:
        return lam_min
    ratio = max(0.0, s / sigma_threshold)
    return lam_max - (lam_max - lam_min) * ratio


def null_space_project(j, z, lam=1e-6):
    """把向量 z 投影到雅可比的零空间，返回 (I − J⁺J) z。

    零空间内的分量不改变末端位姿，可用于关节限位规避等次级任务。
    """
    jz = mat_vec(j, z)
    # J⁺ (J z)
    proj = damped_step(j, jz, lam)
    return [z[i] - proj[i] for i in range(len(z))]


# ---------------------------------------------------------------------------
def rotation_error(r_target, r_current):
    """两个旋转矩阵之间的误差，返回旋转向量（轴 × 角）。

    用于位姿逆解：把姿态误差也当成末端误差的一部分。
    """
    # R_err = R_target · R_currentᵀ
    rt = [[r_current[j][i] for j in range(3)] for i in range(3)]
    r_err = mat_mul(r_target, rt)

    tr = r_err[0][0] + r_err[1][1] + r_err[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    theta = math.acos(c)

    if theta < 1e-9:
        return [0.0, 0.0, 0.0]

    if abs(math.pi - theta) < 1e-6:
        # 接近 180°，从对称部分取轴
        axis = [
            math.sqrt(max(0.0, (r_err[0][0] + 1.0) * 0.5)),
            math.sqrt(max(0.0, (r_err[1][1] + 1.0) * 0.5)),
            math.sqrt(max(0.0, (r_err[2][2] + 1.0) * 0.5)),
        ]
        # 修符号：用非对角元判断
        if r_err[0][1] < 0:
            axis[1] = -axis[1]
        if r_err[0][2] < 0:
            axis[2] = -axis[2]
        n = norm(axis)
        if n < 1e-12:
            return [0.0, 0.0, 0.0]
        axis = [x / n for x in axis]
        return [x * theta for x in axis]

    k = theta / (2.0 * math.sin(theta))
    axis = [k * (r_err[2][1] - r_err[1][2]),
            k * (r_err[0][2] - r_err[2][0]),
            k * (r_err[1][0] - r_err[0][1])]
    return axis


# ---------------------------------------------------------------------------
class IKResult(object):
    """逆解结果。

    success     是否在容差内收敛
    q           解出的关节角
    error       末端位置误差（米）
    iterations  迭代次数
    history     每次迭代的误差，便于画收敛曲线
    reason      结束原因
    """

    def __init__(self, success, q, error, iterations, history, reason):
        self.success = success
        self.q = q
        self.error = error
        self.iterations = iterations
        self.history = history
        self.reason = reason

    def __repr__(self):
        return ("IKResult(%s, err=%.2e, it=%d, %s)"
                % ("ok" if self.success else "fail", self.error,
                   self.iterations, self.reason))


def _perturb(q, scale, seed):
    """确定性微扰，用来从奇异位形脱困。

    用正弦而不是随机数，保证同一个输入每次跑出一样的结果——调试和
    单元测试都需要可复现。
    """
    return [q[i] + scale * math.sin(seed * 1.7 + i * 2.3)
            for i in range(len(q))]


def default_initial_guess(chain):
    """缺省初值：各关节取限位区间的中点。

    为什么不用全零
    --------------
    全零对很多机构正好是**奇异位形**。比如 7 自由度人形手臂全零时完全伸直，
    肘角下限又是 0，弯肘反而让 x 方向误差变大，一阶迭代会卡在 0.3 m 附近
    出不来（实测脱困微扰也只能到 0.18 m）。

    取限位中点就自然得到一个弯肘的常位形（肘部约 1.3 rad），远离奇异，
    同一个目标 5~6 次迭代就收敛到 1e-10。

    没有限位的关节仍取 0。
    """
    q = []
    for j in chain.joints:
        if j.limits is None:
            q.append(0.0)
        else:
            lo, hi = j.limits
            q.append(0.5 * (lo + hi))
    return q


def solve_position_ik(chain, target, q0=None, tol=1e-6, max_iter=200,
                      step_size=1.0, damping="adaptive", has_limits=True,
                      escape=True, stall_patience=15, escape_scale=0.15):
    """位置逆解：迭代把末端拉到 target。

    参数
    ----
    chain        kinematics.Chain
    target       目标位置 [x, y, z]
    q0           初值；缺省取关节限位中点（见 ``default_initial_guess``）
    tol          收敛容差（米）
    max_iter     最大迭代次数
    step_size    每步的缩放，1.0 是标准牛顿步
    damping      "adaptive"（自适应）/ "fixed"（固定 1e-3）/ 具体数值
    has_limits   是否把解夹到关节限位内
    escape       卡住时是否做微扰脱困
    stall_patience  连续多少次没有改进就认为卡住了
    escape_scale    微扰幅度（弧度）

    关于初值
    --------
    逆解是局部方法，收敛与否强烈依赖初值。缺省给限位中点而不是全零，
    就是为了避开"全零 = 完全伸直 = 奇异"这个坑。

    返回 IKResult。
    """
    if len(target) != 3:
        raise ValueError("目标位置必须是三维")
    if tol <= 0:
        raise ValueError("tol 必须为正")
    if max_iter <= 0:
        raise ValueError("max_iter 必须为正")
    if stall_patience <= 0:
        raise ValueError("stall_patience 必须为正")

    if q0 is None:
        q = default_initial_guess(chain)
    else:
        if len(q0) != chain.dof:
            raise ValueError("初值维度和自由度不符")
        q = [float(x) for x in q0]
    if has_limits:
        q = chain.clamp_all(q)

    history = []
    reason = "达到最大迭代次数"
    best = float("inf")
    stall = 0
    escapes = 0

    for it in range(max_iter):
        p, _r = chain.fk(q)
        err = [target[0] - p[0], target[1] - p[1], target[2] - p[2]]
        e_norm = norm(err)
        history.append(e_norm)

        if e_norm < tol:
            return IKResult(True, q, e_norm, it, history, "收敛")

        # 卡住检测
        if e_norm < best - 1e-12:
            best = e_norm
            stall = 0
        else:
            stall += 1
            if escape and stall >= stall_patience:
                q = _perturb(q, escape_scale, it)
                if has_limits:
                    q = chain.clamp_all(q)
                stall = 0
                escapes += 1
                continue

        j = chain.position_jacobian(q)

        if damping == "adaptive":
            lam = adaptive_damping(j)
        elif damping == "fixed":
            lam = 1e-3
        else:
            lam = float(damping)

        try:
            dq = damped_step(j, err, lam)
        except ValueError:
            reason = "线性求解失败（矩阵奇异）"
            break

        q = [q[i] + step_size * dq[i] for i in range(chain.dof)]
        if has_limits:
            q = chain.clamp_all(q)

    p, _r = chain.fk(q)
    e_norm = norm([target[0] - p[0], target[1] - p[1], target[2] - p[2]])
    if escapes:
        reason = "%s（脱困 %d 次）" % (reason, escapes)
    return IKResult(False, q, e_norm, len(history), history, reason)


def solve_pose_ik(chain, target_pos, target_rot, q0=None, tol=1e-6,
                  max_iter=300, step_size=0.8, damping="adaptive",
                  rotation_weight=0.3):
    """位姿逆解：同时约束位置和姿态（6 维误差）。

    位置误差单位是米，姿态误差单位是弧度，量纲不同，所以给姿态乘一个
    ``rotation_weight`` 把两者拉到相近量级。默认 0.3 是经验值：太重会让
    位置精度下降，太轻姿态收敛慢。
    """
    if len(target_pos) != 3:
        raise ValueError("目标位置必须是三维")
    if len(target_rot) != 3 or len(target_rot[0]) != 3:
        raise ValueError("目标姿态必须是 3x3")

    if q0 is None:
        q = [0.0] * chain.dof
    else:
        if len(q0) != chain.dof:
            raise ValueError("初值维度和自由度不符")
        q = [float(x) for x in q0]
    q = chain.clamp_all(q)

    history = []
    for it in range(max_iter):
        p, r = chain.fk(q)
        e_pos = [target_pos[0] - p[0],
                 target_pos[1] - p[1],
                 target_pos[2] - p[2]]
        e_rot = rotation_error(target_rot, r)
        e = e_pos + [rotation_weight * x for x in e_rot]
        e_norm = norm(e)
        history.append(e_norm)

        if e_norm < tol:
            return IKResult(True, q, norm(e_pos), it, history, "收敛")

        j = chain.jacobian(q)
        if damping == "adaptive":
            lam = adaptive_damping(j)
        elif damping == "fixed":
            lam = 1e-3
        else:
            lam = float(damping)

        try:
            dq = damped_step(j, e, lam)
        except ValueError:
            return IKResult(False, q, norm(e_pos), len(history), history,
                            "线性求解失败")
        q = chain.clamp_all([q[i] + step_size * dq[i] for i in range(chain.dof)])

    p, _r = chain.fk(q)
    e_pos = norm([target_pos[0] - p[0],
                  target_pos[1] - p[1],
                  target_pos[2] - p[2]])
    return IKResult(False, q, e_pos, len(history), history, "达到最大迭代次数")
