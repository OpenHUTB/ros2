# -*- coding: utf-8 -*-
"""肌肉冗余系统的力分配：最小范数与加权最小二乘（只用 Python 标准库）。

背景：一个关节（1 个自由度）常被多块肌肉冗余驱动。每块肌肉 i 提供力矩
    tau_i = r_i * (a_i * Fmax_i)
其中 r_i 为力臂（屈肌 +、伸肌 -），a_i 为激活度 [0,1]。

给定期望力矩 tau_des，满足 sum_i tau_i = tau_des 的解有无穷多个。
本模块给出两种经典分配：
    - 最小激活范数（least effort）：min sum a_i^2
    - 加权最小二乘：min sum w_i * a_i^2（可按 Fmax 或疲劳权重调整侧重）

并演示：冗余肌群如何平分负载，以及拮抗共激活如何提高关节刚度。
"""

from __future__ import annotations


class Muscle(object):
    def __init__(self, name, arm, fmax, group):
        self.name = name
        self.arm = float(arm)      # 力臂，正负代表屈/伸
        self.fmax = float(fmax)    # 最大等长力 (N)
        self.group = group         # +1 屈肌 / -1 伸肌


def torque_vector(muscles):
    """R_i = r_i * Fmax_i，即单块肌满激活提供的力矩。"""
    return [m.arm * m.fmax for m in muscles]


def achievable_range(muscles):
    """可行力矩区间 [tau_min, tau_max]（所有屈/伸肌满激活的极值）。"""
    pos = sum(m.arm * m.fmax for m in muscles if m.arm > 0)
    neg = sum(m.arm * m.fmax for m in muscles if m.arm < 0)
    return min(neg, pos), max(neg, pos)


def allocate(tau_des, muscles, weights=None):
    """把期望力矩 tau_des 分配到各肌肉激活度（最小范数/加权最小二乘）。

    只在与期望力矩同向的「有效肌肉」子集上求解，避免算出负的（不可能存在的）
    激活度。返回 (activations, tau_achieved)，激活度裁剪到 [0,1]。
    """
    R = torque_vector(muscles)
    n = len(muscles)
    if weights is None:
        weights = [1.0] * n
    sign = 1 if tau_des >= 0 else -1
    denom = sum(w * Ri * Ri for w, Ri in zip(weights, R) if sign * Ri > 0)
    if denom <= 0:
        raise ValueError("无可参与该方向力矩的肌肉，或力矩向量全为 0")
    acts = []
    for w, Ri in zip(weights, R):
        if sign * Ri > 0:
            acts.append(w * Ri * tau_des / denom)
        else:
            acts.append(0.0)
    acts = [min(1.0, max(0.0, a)) for a in acts]
    tau_ach = sum(Ri * a for Ri, a in zip(R, acts))
    return acts, tau_ach


def joint_stiffness_proxy(muscles, acts):
    """关节刚度代理量：sum |r_i| * Fmax_i * a_i（共激活越大刚度越高）。"""
    R = torque_vector(muscles)
    return sum(abs(Ri) * a for Ri, a in zip(R, acts))


if __name__ == "__main__":
    ms = [
        Muscle("F1", 0.03, 1000, +1),
        Muscle("F2", 0.03, 1000, +1),
        Muscle("E1", -0.03, 1000, -1),
        Muscle("E2", -0.03, 1000, -1),
    ]
    lo, hi = achievable_range(ms)
    print("可行力矩区间: [%.1f, %.1f] Nm" % (lo, hi))
    # 冗余平分：两块屈肌应各承担一半
    a, t = allocate(30.0, ms)
    print("tau_des=30 -> 激活 %s  实现力矩 %.2f" % (["%.3f" % x for x in a], t))
    # 共激活：净力矩为 0，但两块都激活 -> 刚度升高
    a0 = [0.3, 0.3, 0.3, 0.3]
    print("共激活(全0.3) 净力矩 %.2f  刚度代理 %.1f" %
          (sum(torque_vector(ms)[i] * a0[i] for i in range(4)),
           joint_stiffness_proxy(ms, a0)))
