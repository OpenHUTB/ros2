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

    求解的优化问题（w_i > 0 为该肌肉的「代价」权重）：

        min_a  sum_i w_i * a_i^2      s.t.  sum_i R_i * a_i = tau_des,  a_i >= 0

    拉格朗日乘子法（对 a_i 求偏导并令其为零）：

        d/da_i [ sum_j w_j a_j^2 - lam * (sum_j R_j a_j - tau_des) ] = 0
        => 2 w_i a_i = lam R_i  =>  a_i = lam R_i / (2 w_i)

    即 a_i ∝ R_i / w_i —— **权重越大（代价越高）的肌肉分到的激活越小**。
    无约束解析解为

        a_i = tau_des * (R_i / w_i) / sum_{j in S} (R_j^2 / w_j)

    其中 S 为与期望力矩同向的「有效肌肉」子集（R_j * tau_des > 0），这样解出来
    的激活度天然非负，避免把负激活裁剪掉后力矩失真。

    **饱和后的剩余力矩重分配**：若解析解中某块肌肉 a_i > 1（超出物理可行范围），
    不能简单裁剪——那会让实现力矩凭空变小。正确做法是激活集（active-set）迭代：
    把越界的肌肉固定在 a_i = 1，从期望力矩中扣除它单独贡献的 R_i，再用同一个
    最优规则在剩余未饱和肌肉上重解，直到不再出现新的越界。这等价于求解盒约束
    二次规划 min sum w_i a_i^2  s.t.  sum R_i a_i = tau_des, 0 <= a_i <= 1，
    在可行域内能精确实现期望力矩。

    注意权重方向：w 是「代价」，不是「优先度」。若想让某块肌肉优先承担负载，
    应把它的 w_i 取小（例如按 w_i ∝ 1/Fmax_i 让强壮肌肉优先）。

    返回 (activations, tau_achieved)。可行域内 tau_achieved == tau_des；
    超出 achievable_range 时 tau_achieved 饱和在可行边界。
    """
    R = torque_vector(muscles)
    n = len(muscles)
    if weights is None:
        weights = [1.0] * n
    if len(weights) != n:
        raise ValueError("weights 长度必须与 muscles 一致")
    if any(w <= 0.0 for w in weights):
        raise ValueError("weights 必须全部为正（作为代价权重，越大表示越不倾向使用）")
    sign = 1 if tau_des >= 0 else -1
    # 与期望力矩同向的有效肌肉
    eff = [i for i in range(n) if sign * R[i] > 0]
    if not eff:
        raise ValueError("无可参与该方向力矩的肌肉，或力矩向量全为 0")
    acts = [0.0] * n
    fixed = set()                       # 已饱和钉死在 1.0 的肌肉下标
    remaining = float(tau_des)
    while True:
        free = [i for i in eff if i not in fixed]
        denom = sum(R[i] * R[i] / weights[i] for i in free)
        if denom <= 1e-12:
            break                       # 全部饱和：已到可行边界
        lam = remaining / denom
        newly = []
        for i in free:
            a_i = lam * R[i] / weights[i]
            if a_i >= 1.0:
                acts[i] = 1.0
                newly.append(i)
            else:
                acts[i] = max(0.0, a_i)
        if not newly:
            break                       # 无新饱和：解析解本身可行，结束
        fixed.update(newly)
        # 扣除饱和肌肉单独贡献的力矩，在剩余肌肉上重解
        remaining = float(tau_des) - sum(R[i] * acts[i] for i in fixed)
    tau_ach = sum(Ri * a for Ri, a in zip(R, acts))
    return acts, tau_ach


def objective(acts, weights=None):
    """加权目标函数 sum_i w_i a_i^2，便于验证 allocate() 确实取到最小值。"""
    if weights is None:
        weights = [1.0] * len(acts)
    return sum(w * a * a for w, a in zip(weights, acts))


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
