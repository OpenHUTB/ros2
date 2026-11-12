"""膝关节位置阶跃响应的指标计算。

样本为字典序列，键包含:
    t:    阶跃施加后的时间 (s)
    q:    膝关节角 (rad)
    qd:   膝关节角速度 (rad/s)
    tau:  指令力矩 (N·m)
    q_cmd: 阶跃目标角 (rad)

全部指标以「最终稳态值」而不是「指令值」为基准，因为纯 PD 阻抗在
重力负载下有稳态误差，用指令值作基准会把稳态误差混进超调里。

    上升时间   —— 从 10% 到 90%（相对 q(0)→q(∞) 的幅值）所需时间；
    超调量     —— 峰值超出稳态值的比例（相对同一幅值），单位 %；
    调节时间   —— 最后一次离开 ±2% 误差带的时刻；
    稳态误差   —— q_cmd - q(∞)，纯 PD 下约等于 mgl·sin(q)/kp；
    峰值力矩/峰值角速度 —— 执行器与关节的瞬态负荷。

等效惯量与重力矩系数用于给出理论的 ω_n / ζ 作为对照，由 URDF 的质量
分布直接算出（小腿 0.3 kg、质心距膝 0.15 m；足 0.2 kg、质心距膝
0.31 m），不是拍脑袋常数。

所有函数均为纯函数，输入非法时抛出 ValueError，便于单测与批处理复用。
"""

import math

GRAVITY = 9.81

# 膝关节以下连杆绕膝轴的等效转动惯量 (kg·m²)：
#   (0.003 + 0.3*0.15^2) + (0.0006 + 0.2*0.31^2) = 0.0296
EFFECTIVE_INERTIA = 0.0296
# 重力矩线性化系数 m·g·l_com (N·m)：
#   9.81 * (0.3*0.15 + 0.2*0.31) = 1.05
GRAVITY_TORQUE_COEF = 1.05

TAIL_RATIO = 0.10
SETTLE_BAND = 0.02


def _require_samples(samples):
    if not samples:
        raise ValueError('样本序列为空，无法计算阶跃响应指标')


def initial_value(samples):
    """阶跃施加瞬间的起始角。"""
    _require_samples(samples)
    return samples[0]['q']


def final_value(samples, tail_ratio=TAIL_RATIO):
    """稳态值：取序列尾段 tail_ratio 的平均，抑制残余抖动。"""
    _require_samples(samples)
    count = max(1, int(round(len(samples) * tail_ratio)))
    tail = samples[-count:]
    return sum(sample['q'] for sample in tail) / len(tail)


def steady_state_error(samples, command, tail_ratio=TAIL_RATIO):
    """稳态误差 q_cmd - q(∞)，正值表示不到位。"""
    return command - final_value(samples, tail_ratio)


def rise_time(samples, tail_ratio=TAIL_RATIO, low=0.1, high=0.9):
    """上升时间：10% → 90% 幅值所需时间；幅值不足或未达到时返回 None。"""
    _require_samples(samples)
    start = initial_value(samples)
    span = final_value(samples, tail_ratio) - start
    if abs(span) < 1e-6:
        return None
    low_level = start + low * span
    high_level = start + high * span
    t_low = next((s['t'] for s in samples
                  if (s['q'] - low_level) * span >= 0), None)
    t_high = next((s['t'] for s in samples
                   if (s['q'] - high_level) * span >= 0), None)
    if t_low is None or t_high is None:
        return None
    return t_high - t_low


def peak_value(samples):
    """阶跃过程中的最大关节角（正向阶跃）。"""
    _require_samples(samples)
    return max(sample['q'] for sample in samples)


def overshoot(samples, tail_ratio=TAIL_RATIO):
    """超调量（%）：峰值超出稳态值的部分占 q(0)→q(∞) 幅值的比例。"""
    _require_samples(samples)
    start = initial_value(samples)
    final = final_value(samples, tail_ratio)
    span = final - start
    if abs(span) < 1e-6:
        return 0.0
    return max(0.0, (peak_value(samples) - final) / abs(span) * 100.0)


def settling_time(samples, tail_ratio=TAIL_RATIO, band=SETTLE_BAND):
    """调节时间：最后一次离开 ±band 误差带的时刻。

    若序列末尾仍在误差带之外，说明在观测窗口内没有稳定，返回 None，
    避免把「截断时刻」当成「整定完成」。
    """
    _require_samples(samples)
    start = initial_value(samples)
    final = final_value(samples, tail_ratio)
    span = abs(final - start)
    if span < 1e-6:
        return 0.0
    tolerance = band * span
    outside = [s['t'] for s in samples if abs(s['q'] - final) > tolerance]
    if not outside:
        return 0.0
    if abs(samples[-1]['q'] - final) > tolerance:
        return None
    return outside[-1]


def predicted_steady_state_error(kp, command,
                                 coefficient=GRAVITY_TORQUE_COEF,
                                 iterations=60):
    """纯 PD 阻抗在重力负载下的稳态误差预测。

    稳态满足 kp·e = mgl·sin(q_des - e)，对 e 做不动点迭代求解。
    与直接用 mgl·sin(q_des)/kp 的近似相比，kp 较小时不会高估误差
    （sin(q) 随误差增大而减小）。
    """
    if kp <= 0.0:
        raise ValueError('kp 必须为正数')
    error = coefficient / kp
    for _ in range(iterations):
        error = coefficient * math.sin(command - error) / kp
    return error


def peak_velocity(samples):
    """最大关节角速度绝对值 (rad/s)。"""
    _require_samples(samples)
    return max(abs(sample['qd']) for sample in samples)


def peak_torque(samples):
    """最大指令力矩绝对值 (N·m)，用于判断是否触碰限幅。"""
    _require_samples(samples)
    return max(abs(sample['tau']) for sample in samples)


def natural_frequency(kp, inertia=EFFECTIVE_INERTIA):
    """无阻尼自然频率 ω_n = sqrt(kp / J)，单位 rad/s。"""
    if kp <= 0.0:
        raise ValueError('kp 必须为正数')
    if inertia <= 0.0:
        raise ValueError('inertia 必须为正数')
    return math.sqrt(kp / inertia)


def damping_ratio(kp, kd, inertia=EFFECTIVE_INERTIA):
    """阻尼比 ζ = kd / (2·sqrt(kp·J))。"""
    return kd / (2.0 * natural_frequency(kp, inertia) * inertia)


def compute_metrics(samples, command):
    """汇总一次阶跃响应实验的全部指标。"""
    return {
        'initial_value': initial_value(samples),
        'final_value': final_value(samples),
        'steady_state_error': steady_state_error(samples, command),
        'rise_time': rise_time(samples),
        'peak_value': peak_value(samples),
        'overshoot': overshoot(samples),
        'settling_time': settling_time(samples),
        'peak_velocity': peak_velocity(samples),
        'peak_torque': peak_torque(samples),
    }
