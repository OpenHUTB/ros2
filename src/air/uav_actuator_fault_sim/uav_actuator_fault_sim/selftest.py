# -*- coding: utf-8 -*-
"""内置自检：不依赖 ROS，验证故障算子、用例矩阵与仿真闭环是否符合预期。

``python main.py --selftest`` 会跑一遍；作为 CI 的补充（本仓库只有文档构建 CI，
没有测试 CI），提交 PR 前本地跑这条命令就够。
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np

from . import fault as F
from .scenario import FAULT_SCENARIOS, FAULT_ONLY, PHYSICAL_SCENES, full_matrix, training_matrix

_MAX_THRUST = 6.0


def _check_fault_shapes() -> List[Tuple[str, bool, str]]:
    """四种故障的增益/偏置是否落在物理可解释的范围内。"""
    out = []
    inj = F.FaultInjector(F.FaultSpec(rotor_faults=(F.RotorFault(1, "loss", onset=2.0),)),
                          max_thrust=_MAX_THRUST)
    g0 = inj.gains(0.0)
    g1 = inj.gains(3.0)
    out.append(("完全失效：注入前增益为 1", bool(np.allclose(g0, 1.0)), str(g0.tolist())))
    out.append(("完全失效：注入后该桨增益为 0", g1[1] == 0.0, str(g1.tolist())))
    out.append(("完全失效：其余桨不受影响",
                bool(np.allclose(np.delete(g1, 1), 1.0)), str(g1.tolist())))

    inj = F.FaultInjector(
        F.FaultSpec(rotor_faults=(F.RotorFault(0, "degradation", onset=1.0, severity=0.6),)),
        max_thrust=_MAX_THRUST)
    out.append(("推力衰减：增益等于保留比例", abs(inj.gains(2.0)[0] - 0.6) < 1e-9,
                str(inj.gains(2.0).tolist())))

    inj = F.FaultInjector(
        F.FaultSpec(rotor_faults=(F.RotorFault(2, "stuck", onset=1.0, severity=0.5),)),
        max_thrust=_MAX_THRUST)
    out.append(("卡死：增益为 0 且偏置非零",
                inj.gains(2.0)[2] == 0.0 and inj.offsets(2.0)[2] > 0.0,
                f"g={inj.gains(2.0)[2]:.3f} b={inj.offsets(2.0)[2]:.3f}"))

    # progressive 的 severity 语义是"衰减速率 −λ̇（1/s）"，不是"最终增益"；
    # 取 0.2/s 才能让 t=2.0 与 t=3.5 两个采样点都落在 (0, 1) 内便于观察递减。
    inj = F.FaultInjector(
        F.FaultSpec(rotor_faults=(F.RotorFault(3, "progressive", onset=1.0,
                                              severity=0.2, ramp=0.0),)),
        max_thrust=_MAX_THRUST)
    g_mid = inj.gains(2.0)[3]
    g_end = inj.gains(3.5)[3]
    out.append(("渐进退化：增益随时间单调下降", 0.0 < g_end < g_mid < 1.0,
                f"t=2.0 → {g_mid:.3f}, t=3.5 → {g_end:.3f}（0.2/s）"))
    return out


def _check_clipping() -> List[Tuple[str, bool, str]]:
    inj = F.FaultInjector(F.FaultSpec(rotor_faults=(F.RotorFault(0, "stuck", onset=0.0,
                                                               severity=1.0),)),
                          max_thrust=_MAX_THRUST)
    u = np.full(4, 2.0)
    applied = inj.apply(u, 1.0)
    ok = bool(np.all(applied <= _MAX_THRUST + 1e-9)) and bool(np.all(applied >= 0.0))
    return [("注入后推力被裁剪到 [0, T_max]", ok, str(np.round(applied, 3).tolist()))]


def _check_matrix() -> List[Tuple[str, bool, str]]:
    full = full_matrix(seeds=(0,))
    train = training_matrix(seeds=(0, 1, 2))
    expected_full = len(PHYSICAL_SCENES) * len(FAULT_SCENARIOS)
    expected_train = len(PHYSICAL_SCENES) * len(FAULT_ONLY) * 3
    return [
        ("完整矩阵 = 5 场景 × 7 故障", len(full) == expected_full,
         f"{len(full)} / {expected_full}"),
        ("训练矩阵 = 5 场景 × 6 故障 × 3 seed", len(train) == expected_train,
         f"{len(train)} / {expected_train}"),
        ("所有用例 tag 唯一", len({c.tag() for c in full}) == len(full), ""),
    ]


def _check_flight() -> List[Tuple[str, bool, str]]:
    """短时仿真：健康要稳、单桨全失效要掉。"""
    from .runner import RunConfig, run_many
    from .scenario import make_case

    cases = [
        make_case("open_ground", "healthy", seed=0, rotor=0, onset=2.0),
        make_case("open_ground", "single_loss", seed=0, rotor=0, onset=2.0),
    ]
    results = run_many(cases, cfg=RunConfig(duration=6.0))
    healthy, broken = results
    return [
        ("健康飞行 6 s 不坠机", not healthy.crashed,
         f"最低高度 {healthy.min_altitude:.2f} m"),
        ("健康飞行高度保持 1.5 m", abs(healthy.min_altitude - 1.5) < 0.05,
         f"最低高度 {healthy.min_altitude:.3f} m"),
        ("单桨全失效 1 s 内坠机", broken.crashed and (broken.crash_delay or 9) < 1.0,
         f"坠地延时 {broken.crash_delay} s"),
        ("单桨全失效触地速度 > 1 m/s", (broken.touchdown_speed or 0) > 1.0,
         f"触地速度 {broken.touchdown_speed} m/s"),
    ]


def _check_sensor() -> List[Tuple[str, bool, str]]:
    """传感器故障必须真的进入控制回路，而不是只写在结果里。"""
    from .runner import RunConfig, run_case
    from .scenario import make_case

    inj = F.FaultInjector(F.sensor_gyro_bias(0.1, onset=2.0), max_thrust=_MAX_THRUST)
    true_w = np.array([0.0, 0.0, 0.0])

    # 时间没到 onset：读数应等于真值
    before = inj.sense_omega(true_w, 1.0)
    # 过渡期内：介于 0 与满量级之间
    during = inj.sense_omega(true_w, 2.25)
    # 过渡结束后：x 轴等于完整零偏
    after = inj.sense_omega(true_w, 4.0)

    healthy = run_case(make_case("open_ground", "healthy", seed=0), RunConfig(duration=5.0))
    biased = run_case(make_case("open_ground", "sensor_bias", seed=0), RunConfig(duration=5.0))
    drift_gap = abs(biased.horizontal_drift - healthy.horizontal_drift)
    quat_changed = (biased.omega_sensed is not None
                    and float(np.max(np.abs(biased.omega_sensed - biased.omega))) > 0.05)

    return [
        ("故障生效前 IMU 读数等于真值", float(np.max(np.abs(before))) < 1e-12,
         f"|ω| = {float(np.max(np.abs(before))):.2e}"),
        ("过渡期零偏按比例生效", 0.0 < float(during[0]) < 0.1,
         f"ωx = {float(during[0]):.4f} rad/s"),
        ("过渡结束后零偏到位", abs(float(after[0]) - 0.1) < 1e-9,
         f"ωx = {float(after[0]):.4f} rad/s"),
        ("飞行中 IMU 读数被篡改", quat_changed, "max|ω_sensed − ω_true| > 0.05"),
        ("传感器故障改变飞行结果（水平漂移）", drift_gap > 0.05,
         f"健康 {healthy.horizontal_drift:.3f} m → 零偏 {biased.horizontal_drift:.3f} m"),
    ]


def run_selftest(verbose: bool = True) -> int:
    """跑全部自检，返回进程退出码（0 = 全通过）。"""
    groups = [
        ("故障算子", _check_fault_shapes()),
        ("推力裁剪", _check_clipping()),
        ("传感器故障", _check_sensor()),
        ("用例矩阵", _check_matrix()),
        ("飞行闭环", _check_flight()),
    ]
    failures = 0
    for title, checks in groups:
        if verbose:
            print(f"\n[{title}]")
        for name, ok, detail in checks:
            if not ok:
                failures += 1
            if verbose:
                mark = "PASS" if ok else "FAIL"
                extra = f"  ({detail})" if detail else ""
                print(f"  {mark}  {name}{extra}")

    total = sum(len(c) for _, c in groups)
    if verbose:
        print(f"\n自检结果：{total - failures}/{total} 通过")
    return 0 if failures == 0 else 1
