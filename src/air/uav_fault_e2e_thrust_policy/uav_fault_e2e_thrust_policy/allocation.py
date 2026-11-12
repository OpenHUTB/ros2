"""控制分配：把期望的合推力/力矩解成四个旋翼的推力指令。

几何控制器只给出 (T, τ)，真正产生它们的是四个桨的**有效推力**
:math:`v_i = g_i u_i + b_i`（``g`` 是健康度、``b`` 是卡死时的固定输出）。
健康时这是一个 4×4 线性方程组；某个桨坏掉之后，那个桨的 :math:`v_i` 只能取
固定值，剩下的桨还要受 ``0 ≤ v_i ≤ T_max`` 的限制，于是变成带箱约束的最小二乘：

.. math::

    \\min_{\\mathbf v}\\;
        (\\mathbf B\\,\\mathbf v - \\mathbf d)^\\top \\mathbf S
        (\\mathbf B\\,\\mathbf v - \\mathbf d)
        + \\lambda\\lVert\\mathbf v - \\mathbf v_{ref}\\rVert^2
    \\quad\\text{s.t.}\\; 0 \\le v_i \\le g_i T_{max}

其中 :math:`\\mathbf d=[T,\\tau_x,\\tau_y,\\tau_z]^\\top`，
:math:`\\mathbf S=\\mathrm{diag}(w_T,1,1,1)` 给合推力更高权重——力不够时先保高度，
力矩可以少要一点。解出 :math:`\\mathbf v` 后再换算回指令
:math:`u_i = (v_i - b_i)/g_i`。

**注意**：约束是写在**有效推力**上的，上界是 :math:`g_i T_{max}` 而不是
:math:`T_{max}`——衰减到 50% 的桨就算被指令到满量程，也只能出半份推力。
写成"指令不超过 :math:`g_i T_{max}`"是另一个方向的错（那会把健康的桨也限死），
早期版本正是把箱约束按 :math:`T_{max}` 写死，才让单桨衰减也飞成坠机。
"""

from __future__ import annotations

import itertools

import numpy as np

#: 期望量的维数：[合推力, τx, τy, τz]
DESIRED_DIM = 4
ROTOR_COUNT = 4
#: 合推力的权重：比力矩高一个量级，缺力时优先保高度
THRUST_WEIGHT = 20.0
#: 正则项权重（让解唯一、并且贴近上一拍的解，动作更平滑）
REF_WEIGHT = 1e-3
#: 判定"这个桨还能不能控制"的阈值
GAIN_EPS = 1e-6
#: 箱约束判定的容差
BOUND_TOL = 1e-9


def build_effectiveness(mixer, params, eps: float = 1e-6) -> np.ndarray:
    """求控制效率矩阵 ``B``（4×4），满足 ``[T, τx, τy, τz] = B @ v``。

    用底座的 ``mixer`` 在悬停总推力附近做差分探针反推，而不是手写符号——
    这样即使以后底座的混控符号或桨的排布改了，本矩阵也会自动跟上。
    """
    t0 = 4.0 * float(params.hover_thrust)

    def probe(thrust: float, tau: np.ndarray) -> np.ndarray:
        return np.asarray(mixer(thrust, np.asarray(tau, dtype=float), params),
                          dtype=float)

    base = probe(t0, np.zeros(3))
    m = np.zeros((DESIRED_DIM, ROTOR_COUNT))
    m[:, 0] = (probe(t0 + eps, np.zeros(3)) - base) / eps
    for axis in range(3):
        tau = np.zeros(3)
        tau[axis] = eps
        m[:, axis + 1] = (probe(t0, tau) - base) / eps
    return np.linalg.inv(m)


def effective_thrust(u, gains, bias=None) -> np.ndarray:
    """由指令算实际有效推力 ``v = g⊙u + b``。"""
    g = np.asarray(gains, dtype=float)
    b = np.zeros(ROTOR_COUNT) if bias is None else np.asarray(bias, dtype=float)
    return g * np.asarray(u, dtype=float) + b


def command_from_effective(v, params, gains, bias=None) -> np.ndarray:
    """有效推力 → 指令：``u = (v − b)/g``，失效桨（g≈0）给 0。"""
    g = np.asarray(gains, dtype=float)
    b = np.zeros(ROTOR_COUNT) if bias is None else np.asarray(bias, dtype=float)
    t_max = float(params.max_thrust)
    u = np.zeros(ROTOR_COUNT)
    free = g > GAIN_EPS
    u[free] = (np.asarray(v, dtype=float)[free] - b[free]) / g[free]
    return np.clip(u, 0.0, t_max)


def clamp_thrusts(u, params) -> np.ndarray:
    """指令的物理上限就是单桨满推力，与健康度无关。"""
    return np.clip(np.asarray(u, dtype=float), 0.0, float(params.max_thrust))


def naive_redistribute(u_honest, params, gains) -> np.ndarray:
    """朴素补偿：按坏掉的**桨数**把丢掉的推力平均分给其余桨。

    这是"不改控制、只补力"的做法：它假设坏桨完全不产生推力，把那一份平摊出去，
    既破坏了力矩平衡，也不区分"衰减 60%"与"彻底失效"。用来当基线。
    """
    g = np.clip(np.asarray(gains, dtype=float), 0.0, 1.0)
    u = clamp_thrusts(u_honest, params)
    healthy = g > 0.5
    if healthy.any():
        lost = float(np.sum(u[~healthy])) if (~healthy).any() else 0.0
        if lost > 0.0:
            u[healthy] += lost / float(healthy.sum())
    return clamp_thrusts(u, params)


def _weights(row_weights=None) -> np.ndarray:
    if row_weights is None:
        return np.diag([THRUST_WEIGHT, 1.0, 1.0, 1.0])
    return np.diag(np.asarray(row_weights, dtype=float))


def _split_problem(B, desired, params, gains, bias=None, v_ref=None,
                   row_weights=None, ref_weight: float = REF_WEIGHT):
    """把问题拆成"可控桨的未知量"与"失效桨的固定量"。"""
    d = np.asarray(desired, dtype=float)
    g = np.asarray(gains, dtype=float)
    b = np.zeros(ROTOR_COUNT) if bias is None else np.asarray(bias, dtype=float)
    t_max = float(params.max_thrust)

    free = g > GAIN_EPS
    fixed_v = np.clip(b, 0.0, t_max)
    # 有效推力的上界是 g·T_max：坏桨的满量程只剩 g 倍
    upper = np.where(free, np.clip(g, 0.0, 1.0) * t_max, 0.0)

    # 固定部分对期望量的贡献
    d_eff = d - B @ np.where(free, 0.0, fixed_v)

    s = _weights(row_weights)
    bf = B[:, free]
    h = bf.T @ s @ bf + ref_weight * np.eye(int(free.sum()))
    v_ref_f = None
    if v_ref is not None:
        v_ref_f = np.asarray(v_ref, dtype=float)[free]
    return d_eff, bf, s, h, free, fixed_v, t_max, v_ref_f, upper


def ls_allocation(B: np.ndarray, desired, params, gains,
                  v_ref=None, row_weights=None,
                  ref_weight: float = REF_WEIGHT, bias=None) -> np.ndarray:
    """忽略箱约束的加权最小二乘（"加权伪逆"），返回**指令** ``u``。"""
    d_eff, bf, s, h, free, fixed_v, t_max, v_ref_f, upper = _split_problem(
        B, desired, params, gains, bias, v_ref, row_weights, ref_weight)

    rhs = bf.T @ s @ d_eff
    if v_ref_f is not None:
        rhs = rhs + ref_weight * v_ref_f
    v_free = np.linalg.solve(h, rhs) if free.any() else np.zeros(0)
    if free.any():
        v_free = np.clip(v_free, 0.0, upper[free])

    v = fixed_v.copy()
    v[free] = v_free
    return command_from_effective(v, params, gains, bias)


def solve_box_qp(B: np.ndarray, desired, params, gains,
                 v_ref=None, row_weights=None,
                 ref_weight: float = REF_WEIGHT, bias=None,
                 max_combinations: int = 10_000) -> np.ndarray:
    """枚举有效集的精确解（可控桨数 ≤ 4，最多 3⁴ = 81 种），返回**指令** ``u``。

    4 个变量、每个只有"贴下界 / 自由 / 贴上界"三种状态，比迭代求解稳定：
    不会出现不收敛或落到不同局部解的问题。
    """
    d = np.asarray(desired, dtype=float)
    g = np.asarray(gains, dtype=float)
    b = np.zeros(ROTOR_COUNT) if bias is None else np.asarray(bias, dtype=float)
    t_max = float(params.max_thrust)
    s = _weights(row_weights)
    v_ref_full = np.zeros(ROTOR_COUNT) if v_ref is None else np.asarray(v_ref, dtype=float)
    v_ref_full = np.clip(v_ref_full, 0.0, t_max)

    free = g > GAIN_EPS
    fixed_v = np.clip(b, 0.0, t_max)
    upper_full = np.where(free, np.clip(g, 0.0, 1.0) * t_max, 0.0)
    upper = upper_full[free]
    k = int(free.sum())
    if k == 0:
        return command_from_effective(fixed_v, params, gains, bias)
    if 3 ** k > max_combinations:  # pragma: no cover - 本工程只有 4 个桨
        return ls_allocation(B, desired, params, gains, v_ref, row_weights,
                             ref_weight, bias)

    bf = B[:, free]
    h = bf.T @ s @ bf + ref_weight * np.eye(k)
    # 失效桨的固定推力先扣掉，剩下的才是可控桨要凑的量
    fixed_contrib = B[:, ~free] @ fixed_v[~free] if (~free).any() else np.zeros(DESIRED_DIM)
    rhs = bf.T @ s @ (d - fixed_contrib) + ref_weight * v_ref_full[free]

    def cost_of(v_full: np.ndarray) -> float:
        r = B @ v_full - d
        return float(r @ s @ r + ref_weight * float(np.sum((v_full - v_ref_full) ** 2)))

    best = None
    best_cost = np.inf
    for states in itertools.product((0, 1, 2), repeat=k):
        x = np.zeros(k)
        interior = []
        for i, state in enumerate(states):
            if state == 2:
                x[i] = upper[i]
            elif state == 1:
                interior.append(i)

        if interior:
            idx = np.asarray(interior)
            h_ff = h[np.ix_(idx, idx)]
            rhs_f = rhs[idx] - h[np.ix_(idx, np.arange(k))] @ x
            try:
                x[idx] = np.linalg.solve(h_ff, rhs_f)
            except np.linalg.LinAlgError:  # pragma: no cover
                continue
            if not np.all(np.isfinite(x[idx])):
                continue
            if np.any(x[idx] < -BOUND_TOL) or np.any(x[idx] > upper[idx] + BOUND_TOL):
                continue

        x = np.clip(x, 0.0, upper)
        v_full = fixed_v.copy()
        v_full[free] = x
        cost = cost_of(v_full)
        if cost < best_cost:
            best_cost = cost
            best = x

    if best is None:  # pragma: no cover - 理论上不可达
        return ls_allocation(B, desired, params, gains, v_ref, row_weights,
                             ref_weight, bias)
    v = fixed_v.copy()
    v[free] = best
    return command_from_effective(v, params, gains, bias)


def achievable_thrust(params, gains) -> float:
    """当前健康度下四桨能提供的最大合推力（N），卡死偏差不计入。"""
    g = np.clip(np.asarray(gains, dtype=float), 0.0, 1.0)
    return float((g * float(params.max_thrust)).sum())
