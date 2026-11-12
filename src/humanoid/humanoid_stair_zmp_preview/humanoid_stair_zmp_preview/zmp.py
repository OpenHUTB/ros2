# -*- coding: utf-8 -*-
"""ZMP 预览控制（Kajita 2003）—— 纯 numpy，可离线单测。

模型：线性倒立摆（LIPM）
    p = x - (z_c / g) * ẍ
其中 p 是 ZMP，x 是 CoM 水平位置，z_c 是质心高度。

离散化后状态 ``[x, ẋ, ẍ]``，控制量取 jerk ``u``：

    A = [[1, dt, dt²/2], [0, 1, dt], [0, 0, 1]]
    B = [dt³/6, dt²/2, dt]
    C = [1, 0, -(z_c/g)]

控制律 = 状态反馈 + 未来 ZMP 参考的前视项：

    u_k = -K·s_k + Σ_{i=1..N} f_i · p_ref[k+i]

``K`` 与 ``f_i`` 由离散 Riccati 方程解出（迭代到稳态）。前视项是「预览」的核心：
只看当前 ZMP 误差会滞后，必须提前 N 步看到落脚点变化。

这个文件**不 import mujoco**，因此可以在没有仿真器的环境里验证数学。
"""

import numpy as np

G = 9.81


def _dare(A, B, Q, R, iters=20000, tol=1e-14):
    """求解离散代数 Riccati 方程，返回 (P, K)。

    优先用 numpy 自带的 ``solve_discrete_are``（迭代法的定点求解器）。
    早期版本手写标量迭代，在增广系统上**没有收敛到镇定解** —— 表现是闭环矩阵
    Acl 仍有 |λ|>=1，前视增益 f 随步数递增而不是衰减，预览完全失效。
    """
    try:
        # scipy 有正规 DARE 求解器；只在「生成增益」时用，运行时可选
        from scipy.linalg import solve_discrete_are
        P = solve_discrete_are(A, B, Q, R)
    except Exception:
        # 退化情形兜底：标量迭代
        P = Q.copy()
        for _ in range(iters):
            BtP = B.T @ P
            S = R + BtP @ B
            Kk = np.linalg.solve(S, BtP @ A)
            P_new = Q + A.T @ P @ (A - B @ Kk)
            if np.max(np.abs(P_new - P)) < tol:
                P = P_new
                break
            P = P_new
    BtP = B.T @ P
    K = np.linalg.solve(R + BtP @ B, BtP @ A)
    return P, K


def _augmented(z_c, dt):
    """LIPM + ZMP 误差积分 的增广状态空间（4 维）。

    状态 X = [x, ẋ, ẍ, e]ᵀ，e 是 ZMP 跟踪误差的累积：
        e_{k+1} = e_k + (p_k - p_ref_k) = e_k + C x_k - p_ref_k
    因此
        A_aug = [[A, 0], [C, 1]]，B_aug = [B; 0]，p_ref 从 e 那一行进入。
    """
    A = np.array([[1.0, dt, 0.5 * dt * dt],
                  [0.0, 1.0, dt],
                  [0.0, 0.0, 1.0]])
    B = np.array([[dt ** 3 / 6.0], [dt ** 2 / 2.0], [dt]])
    C = np.array([[1.0, 0.0, -z_c / G]])
    A_aug = np.zeros((4, 4)); A_aug[:3, :3] = A; A_aug[3, :3] = C[0]; A_aug[3, 3] = 1.0
    B_aug = np.zeros((4, 1)); B_aug[:3, 0] = B[:, 0]
    return A_aug, B_aug, C


def preview_gains(z_c=0.80, dt=0.004, n_preview=160,
                  q_x=1.0, q_e=1.0, r=1e-2):
    """返回 (K_aug(4,), f(N,))。

    关键点：**必须对增广系统求解**。早期版本用「3 维状态 LQR 的 K + 独立推导的前视项」，
    两者不是同一个 Riccati 解，闭环直接发散（实测 ZMP 误差 2.2 m 量级）。
    """
    A_aug, B_aug, C = _augmented(z_c, dt)
    Q = np.zeros((4, 4))
    Q[0, 0] = q_x          # CoM 偏移
    Q[3, 3] = q_e          # ZMP 误差累积（主导项）
    R = np.array([[r]])
    P, K = _dare(A_aug, B_aug, Q, R)

    BtP = B_aug.T @ P
    inv = np.linalg.inv(R + BtP @ B_aug)      # 1x1
    Acl = A_aug - B_aug @ K
    # 前视增益的分子必须是 P·E，而不是 Q·e4。
    # E 是 ZMP 参考进入系统的通道：x_{k+1} = A x + B u - E p_ref，E = e4。
    # 用 Q·e4 会让 |f| 比 |K| 小四个数量级，前视实际上等于没开（实测 RMS 125.3 -> 124.9 mm，
    # 开与关几乎相同）。
    E = np.zeros((4, 1)); E[3, 0] = 1.0
    vec = P @ E
    f = np.zeros(n_preview)
    for i in range(n_preview):
        f[i] = float((inv @ (B_aug.T @ (np.linalg.matrix_power(Acl.T, i) @ vec)))[0, 0])
    return K.reshape(-1), f


def simulate(zmp_ref, z_c=0.80, dt=0.004, n_preview=160,
             q_x=1.0, q_e=1.0, r=1e-2, jerk_limit=5.0e3, use_preview=True):
    """按 ZMP 参考跑闭环，返回 com / zmp / zmp_ref / u 等。

    ``jerk_limit`` 是数值保护：增广系统在极端权重下仍可能给出过大的 jerk。
    """
    K, f = preview_gains(z_c, dt, n_preview, q_x, q_e, r)
    A_aug, B_aug, C = _augmented(z_c, dt)

    n = len(zmp_ref)
    X = np.array([zmp_ref[0], 0.0, 0.0, 0.0])
    com = np.zeros(n); vel = np.zeros(n); acc = np.zeros(n); us = np.zeros(n)
    errs = np.zeros(n)
    pad = np.concatenate([zmp_ref, np.full(n_preview + 2, zmp_ref[-1])])
    for k in range(n):
        ff = (f @ pad[k + 1:k + 1 + n_preview]) if use_preview else 0.0
        u = float(-K @ X + ff)
        u = float(np.clip(u, -jerk_limit, jerk_limit))
        p_ref = zmp_ref[k]
        # 增广状态推进：x_{k+1}=A x + B u；e_{k+1}=e + (Cx - p_ref)
        X = A_aug @ X + (B_aug * u).reshape(-1)
        X[3] -= p_ref
        com[k] = X[0]; vel[k] = X[1]; acc[k] = X[2]; errs[k] = X[3]; us[k] = u
    zmp_calc = (C @ np.vstack([com, vel, acc])).reshape(-1)
    return dict(com=com, com_vel=vel, com_acc=acc, zmp=zmp_calc,
                zmp_ref=zmp_ref, u=us, dt=dt, err_int=errs)


def staircase_zmp(n_steps=3, step_duration=0.8, dt=0.004, shift=0.10,
                  com_y=0.0, total_time=None):
    """生成「左右脚交替」的 ZMP 参考（近似方波，模拟落脚点切换）。"""
    T = total_time or (n_steps + 2) * step_duration
    n = int(round(T / dt))
    ref = np.zeros(n)
    for k in range(n):
        t = k * dt
        idx = int(t / step_duration)
        ref[k] = -shift if idx % 2 == 0 else shift
    return ref