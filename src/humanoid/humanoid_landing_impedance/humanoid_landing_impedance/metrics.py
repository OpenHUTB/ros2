"""落地实验序列的指标计算。

样本为字典序列，键包含:
    t: 仿真时间 (s)
    grf: 双脚地面反力合力 (N)
    base_vz: 底座垂直速度 (m/s)
    base_z: 底座高度 (m)
    base_pitch: 底座俯仰角 (rad)
    knee_flexion: 左膝关节角 (rad)

所有函数均为纯函数，输入非法时抛出 ValueError 或返回 None，
便于单元测试与批处理复用。
"""

GRAVITY = 9.81
TOUCHDOWN_GRF_THRESHOLD = 1.0
ACTIVE_PEAK_DELAY = 0.05
IMPULSE_WINDOW = 0.30
SETTLE_VZ_TOL = 0.05
SETTLE_GRF_TOL_RATIO = 0.20


def find_touchdown(samples, threshold=TOUCHDOWN_GRF_THRESHOLD):
    """返回首次触地时间；从未触地则返回 None。"""
    for sample in samples:
        if sample['grf'] > threshold:
            return sample['t']
    return None


def first_peak_grf(samples):
    """全程最大地面反力（被动冲击峰，主要由接触力学决定）。"""
    if not samples:
        raise ValueError('样本序列为空')
    return max(sample['grf'] for sample in samples)


def active_peak_grf(samples, touchdown, delay=ACTIVE_PEAK_DELAY):
    """触地 delay 秒之后的最大地面反力（主动峰，反映身体减速过程）。"""
    if touchdown is None:
        return None
    window = [s for s in samples if s['t'] >= touchdown + delay]
    if not window:
        return None
    return max(s['grf'] for s in window)


def loading_impulse(samples, touchdown, window=IMPULSE_WINDOW):
    """触地后 window 秒内的地面反力冲量（梯形积分，用于动量守恒校验）。"""
    if touchdown is None:
        return None
    segment = [s for s in samples if touchdown <= s['t'] <= touchdown + window]
    if len(segment) < 2:
        return None
    impulse = 0.0
    for prev, cur in zip(segment[:-1], segment[1:]):
        impulse += 0.5 * (prev['grf'] + cur['grf']) * (cur['t'] - prev['t'])
    return impulse


def max_knee_flexion(samples):
    """落地过程中的最大膝关节角（缓冲行程）。"""
    if not samples:
        raise ValueError('样本序列为空')
    return max(sample['knee_flexion'] for sample in samples)


def max_pitch(samples):
    """落地过程中的最大俯仰角偏移幅值。"""
    if not samples:
        raise ValueError('样本序列为空')
    return max(abs(sample['base_pitch']) for sample in samples)


def settle_time(samples, touchdown, mass,
                vz_tol=SETTLE_VZ_TOL, grf_tol_ratio=SETTLE_GRF_TOL_RATIO):
    """恢复时间：触地后最后一次超出稳定判据的时刻。

    稳定判据: |vz| <= vz_tol 且 |grf - 静重| <= grf_tol_ratio * 静重。
    """
    if touchdown is None:
        return None
    static_weight = mass * GRAVITY
    settle = touchdown
    for sample in samples:
        if sample['t'] <= touchdown:
            continue
        if (abs(sample['base_vz']) > vz_tol
                or abs(sample['grf'] - static_weight)
                > grf_tol_ratio * static_weight):
            settle = sample['t']
    return settle


def compute_metrics(samples, mass):
    """汇总一次落地的全部指标，返回字典。"""
    touchdown = find_touchdown(samples)
    return {
        'touchdown': touchdown,
        'first_peak_grf': first_peak_grf(samples),
        'active_peak_grf': active_peak_grf(samples, touchdown),
        'loading_impulse': loading_impulse(samples, touchdown),
        'max_knee_flexion': max_knee_flexion(samples),
        'max_pitch': max_pitch(samples),
        'settle_time': settle_time(samples, touchdown, mass),
    }
