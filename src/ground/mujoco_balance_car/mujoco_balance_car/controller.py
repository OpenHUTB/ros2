#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双环级联控制：位置 PID（外环）→ 目标倾角，倾角 PID（内环）→ 电机力矩。

为什么用级联而不是单环：

* 单环直接把"位置误差"映射成力矩，本质上只能把车推向目标，没法同时保证
  **摆杆不倒**——摆杆倒下去以后位置误差再小也没意义。
* 级联把两件事分开：外环只决定"往哪个方向倾、倾多少"（输出被限幅在 ±0.15 rad，
  相当于限速），内环负责"把倾角追到这个期望值"。倒立摆的平衡靠内环，位置保持
  靠外环慢慢把倾角目标往回收。

符号约定（实测踩过的坑）：模型里底盘只能沿 x 平移，电机给车轮正力矩时车体会向
x 负方向加速，所以 env 里对力矩取了一次反号（`MOTOR_SIGN=-1`）。控制器本身始终
用"正误差 → 正倾角目标"的直观符号，反号只存在于最底层，避免上层出现"调出来的负号"。
"""

from mujoco_balance_car.config import BalanceConfig
from mujoco_balance_car.pid import PIDController


class BalanceController:
    """双环 PID 控制器。"""

    def __init__(self, cfg=None, target_position=None):
        self.cfg = cfg or BalanceConfig()
        self.target_position = (self.cfg.target_position if target_position is None
                                else float(target_position))
        self.angle_pid = PIDController(
            kp=self.cfg.kp_angle, ki=self.cfg.ki_angle, kd=self.cfg.kd_angle,
            output_min=self.cfg.torque_min, output_max=self.cfg.torque_max,
            # 内环也要对测量值求导：外环给出的目标倾角是变化的，若对"误差"求导，
            # d(lean_cmd)/dt 会混进微分项，在位置误差快速变化时踢出力矩尖峰，
            # 实测会导致慢速振荡（周期约 10 s、幅度逐渐增大）。
            # 对测量值求导后等价于全状态 PD：ctrl = kθ·θ + kω·θ̇ + kx·(x−x*) + kv·ẋ
            derivative_on_measurement=True)
        self.pos_pid = PIDController(
            kp=self.cfg.kp_pos, ki=self.cfg.ki_pos, kd=self.cfg.kd_pos,
            output_min=self.cfg.lean_min, output_max=self.cfg.lean_max,
            # 目标是阶跃信号：微分对测量值求导，避免目标突变踢出一个力矩尖峰
            derivative_on_measurement=True)

    # ---------------------------------------------------------------- 接口
    def reset(self):
        self.angle_pid.reset()
        self.pos_pid.reset()

    def compute(self, x, angle, dt):
        """返回 `(torque, target_lean)`：力矩与此刻外环给出的目标倾角。"""
        target_lean = self.pos_pid.compute(self.target_position, x, dt)
        torque = self.angle_pid.compute(target_lean, angle, dt)
        return torque, target_lean

    def set_target(self, target_position):
        """改位置目标（阶跃测试用）。"""
        self.target_position = float(target_position)
