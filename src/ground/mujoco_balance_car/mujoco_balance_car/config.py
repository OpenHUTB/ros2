#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""参数集中定义。

为什么单独抽一个文件：本模块的指标（稳定时间、超调、稳态误差）对 PID 增益非常
敏感，调参时如果增益散落在 main.py 里，既容易改漏也没法写单元测试。这里用
dataclass 固化默认值，CLI 的 `--kp-angle` 等参数只是覆盖它。
"""

from dataclasses import dataclass, field


@dataclass
class BalanceConfig:
    """双环 PID 参数与仿真参数。

    这组默认值的来源（不是"随便调一调"）：级联控制展开后等价于全状态 PD

        ctrl = kθ·θ + kω·θ̇ + kx·(x − x*) + kv·ẋ

    我们在四维增益空间里做网格搜索（kθ∈{10,20,30,40}、kω∈{1,2,3.5,5}、
    kx∈{1,3,6,12}、kv∈{0.5,1.5,3,6}，判据见 test/test_balance_logic.py），
    最优解 kθ=30, kω=3.5, kx=6, kv=6 反解回级联形式：

        kp_angle=30, kd_angle=3.5, kp_pos=kx/kθ=0.2, kd_pos=kv/kθ=0.2

    实测：倾角 0.19 s 进带、位置 0.84 s 进带、1 m 阶跃 2.95 s 到位（超调 3.2%）。
    原实现里的 12/3.5 只闭合了倾角环、没有位置积分/合适的外环增益，在本模型上
    会留下约 0.086 rad 的静态倾角并缓慢振荡（见 README「调参记录」）。
    """

    # --- 倾角内环：误差单位 rad，输出单位 N·m（电机力矩）---
    kp_angle: float = 30.0
    ki_angle: float = 0.0
    kd_angle: float = 3.5
    torque_min: float = -2.0
    torque_max: float = 2.0

    # --- 位置外环：误差单位 m，输出是"期望倾角"（rad）---
    kp_pos: float = 0.2
    ki_pos: float = 0.0
    kd_pos: float = 0.2
    lean_min: float = -0.25
    lean_max: float = 0.25

    # --- 仿真参数 ---
    duration: float = 6.0          # 单场景仿真时长（s）
    control_dt: float = 0.002      # 控制周期（s），与模型 timestep 一致
    target_position: float = 0.0   # 位置目标（m）
    initial_lean: float = 0.05     # 初始倾角（rad），0.05≈2.9°

    # --- 判据 ---
    angle_tol: float = 0.01        # 倾角稳定判据（rad，≈0.57°）
    pos_tol: float = 0.02          # 位置稳定判据（m）
    steady_frac: float = 0.10      # 取末段 10% 计算稳态误差


@dataclass
class ScenarioConfig:
    """批量实验的场景设置。"""

    lean_sweep: tuple = (0.02, 0.05, 0.08, 0.12, 0.16, 0.20, 0.25, 0.30)
    step_position: float = 1.0     # 位置阶跃幅值（m）
    step_time: float = 1.0         # 阶跃发生时刻（s）
    disturbance_time: float = 2.0  # 施加扰动的时刻（s）
    disturbance_velocity: float = 1.5  # 扰动速度（m/s），直接加到 qvel[0]
    settle_window: float = 6.0     # 每个场景的仿真时长（s）
    extra: dict = field(default_factory=dict)
