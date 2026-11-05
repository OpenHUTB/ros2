# -*- coding: utf-8 -*-
"""
evaluate.py —— 抗扰恢复性能评价与临界冲量测定（无 ROS 依赖）

对外接口：
  * ``run_trial``      单次扰动实验：施加一次推力，返回最大质心偏移、恢复时间、
                       是否跌倒、恢复后姿态偏差等指标；
  * ``critical_force`` 二分法测定给定方向上的「最大可恢复力」与对应冲量；
  * ``sweep_forces``   按力序列逐点测量，用于绘制「扰动力 → 最大偏移 / 是否恢复」曲线；
  * ``compare_strategies`` 对比「仅踝策略」与「踝+髋策略」的临界冲量。

恢复判据（与文档一致）：
  扰动结束后，质心水平位移回到 5 mm 以内 **且** 质心速度小于 0.02 m/s，
  连续满足即判定恢复，取首次满足时刻为恢复时间。
"""

import numpy as np

from .balance import BalanceController
from .sim_core import BalanceSim

#: 默认扰动持续时间 (s)
PUSH_DURATION = 0.08
#: 恢复判据阈值
RECOVER_OFFSET = 0.005      # m
RECOVER_SPEED = 0.02        # m/s


def make_sim(params, balance=True):
    """按参数字典构造仿真与控制器。"""
    sim = BalanceSim(params=params)
    if balance:
        sim.controller = BalanceController(params)
    sim.reset()
    return sim


def run_trial(params, force=120.0, direction_deg=0.0, hip_enable=None,
              duration=None, settle=1.0, total=8.0, balance=True):
    """单次扰动实验。

    参数:
        params:        参数字典（见 config/params.yaml）
        force:         扰动力大小 (N)
        direction_deg: 扰动方向（0° = +x 前向，90° = +y，180° = −x 后向）
        hip_enable:    是否启用髋策略；None 表示用 params 中的取值
        duration:      扰动持续时间 (s)；None 表示用 params 中的取值
        settle:        施加扰动前的稳定时间 (s)
        total:         总仿真时长 (s)
        balance:       是否启用平衡控制器（False 用于「无平衡」对照）
    """
    p = dict(params)
    if hip_enable is not None:
        p["hip_enable"] = bool(hip_enable)
    duration = float(p.get("push_duration", PUSH_DURATION) if duration is None else duration)

    sim = make_sim(p, balance=balance)
    ref = sim.com_position()[:2].copy()
    ns = int(settle / sim.dt)
    nt = int(total / sim.dt)

    rad = np.radians(float(direction_deg))
    fx, fy = float(force) * np.cos(rad), float(force) * np.sin(rad)

    max_offset = 0.0
    recover_time = None
    settle_offset = None
    fell = False
    pushed = False

    for i in range(nt):
        if i == ns:
            sim.apply_push(force=(fx, fy, 0.0), duration=duration)
            pushed = True
        sim.step(balance=balance)
        offset = float(np.linalg.norm(sim.com_position()[:2] - ref))
        if pushed:
            max_offset = max(max_offset, offset)
        if sim.fallen():
            fell = True
            break
        if pushed and i > ns + int(0.15 / sim.dt):
            speed = float(np.linalg.norm(sim.com_velocity()[:2]))
            if recover_time is None and offset < RECOVER_OFFSET and speed < RECOVER_SPEED:
                recover_time = (i - ns) * sim.dt
                settle_offset = offset

    state = sim.read_state()
    result = {
        "force": float(force),
        "direction_deg": float(direction_deg),
        "duration": duration,
        "impulse": float(force) * duration,
        "hip_enable": bool(p.get("hip_enable", True)),
        "balance": bool(balance),
        "fell": fell,
        "recovered": (not fell) and (recover_time is not None),
        "max_offset_m": max_offset,
        "recover_time_s": recover_time,
        # 恢复后残差：质心相对初始参考的偏移与躯干倾角
        "residual_offset_m": float(np.linalg.norm(sim.com_position()[:2] - ref)) if not fell else None,
        "final_tilt_rad": state["tilt"],
        "final_root_height": state["root_height"],
    }
    sim.close()
    return result


def critical_force(params, direction_deg=0.0, hip_enable=None,
                   lo=10.0, hi=600.0, iterations=14, check_bounds=True, **kwargs):
    """二分法测定给定方向上的最大可恢复力 (N)。

    ``lo`` 为已知可恢复的力，``hi`` 为已知会跌倒的力；二分 ``iterations`` 次后
    返回可恢复的最大力，精度约 ``(hi-lo)/2**iterations``。

    ``check_bounds=True``（默认）会先验证两个边界假设成立——二分法只有在
    「lo 可恢复、hi 会跌倒」时才收敛到临界值，否则结果无意义，因此这里做了前置校验，
    不成立时直接抛出 ``ValueError`` 而不是返回一个看似合理的错误数字。
    """
    if check_bounds:
        if run_trial(params, force=lo, direction_deg=direction_deg,
                     hip_enable=hip_enable, **kwargs)["fell"]:
            raise ValueError(
                f"二分法下界不成立：{lo} N 在方向 {direction_deg}° 下未能恢复，"
                f"请调小 lo 或检查参数")
        if not run_trial(params, force=hi, direction_deg=direction_deg,
                         hip_enable=hip_enable, **kwargs)["fell"]:
            raise ValueError(
                f"二分法上界不成立：{hi} N 在方向 {direction_deg}° 下竟然恢复，"
                f"请调大 hi")
    for _ in range(int(iterations)):
        mid = 0.5 * (lo + hi)
        if run_trial(params, force=mid, direction_deg=direction_deg,
                     hip_enable=hip_enable, **kwargs)["fell"]:
            hi = mid
        else:
            lo = mid
    return lo


def sweep_forces(params, forces, direction_deg=0.0, hip_enable=None, **kwargs):
    """按给定力序列逐点测量，返回结果列表（用于绘制扰动力-偏移曲线）。"""
    return [run_trial(params, force=float(f), direction_deg=direction_deg,
                      hip_enable=hip_enable, **kwargs) for f in forces]


def compare_strategies(params, directions=(0.0, 90.0), **kwargs):
    """对比「仅踝策略」与「踝+髋策略」的临界冲量。

    返回 ``{"ankle_only": {...}, "ankle_plus_hip": {...}, "gain_percent": {...}}``。
    """
    out = {"ankle_only": {}, "ankle_plus_hip": {}, "gain_percent": {}}
    for deg in directions:
        f_ankle = critical_force(params, direction_deg=deg, hip_enable=False, **kwargs)
        f_hip = critical_force(params, direction_deg=deg, hip_enable=True, **kwargs)
        key = f"{deg:.0f}"
        out["ankle_only"][key] = {"critical_force_n": f_ankle,
                                  "critical_impulse_ns": f_ankle * params.get("push_duration", PUSH_DURATION)}
        out["ankle_plus_hip"][key] = {"critical_force_n": f_hip,
                                      "critical_impulse_ns": f_hip * params.get("push_duration", PUSH_DURATION)}
        out["gain_percent"][key] = (f_hip - f_ankle) / f_ankle * 100.0 if f_ankle > 0 else 0.0
    return out
