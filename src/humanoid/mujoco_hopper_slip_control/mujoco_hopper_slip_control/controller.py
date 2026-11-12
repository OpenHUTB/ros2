# -*- coding: utf-8 -*-
"""弹跳控制器 —— 纯 Python，不 import mujoco，便于离线单元测试。

控制分相
--------
飞行相
    * 腿长用 PD 收回标称长度，为着陆做准备（避免伸直的腿硬砸地面）；
    * 髋关节打到**顶点处算定**的落点角 θ_td。
支撑相
    * 髋关节自由（指令跟随当前角，伺服只留阻尼），腿绕支撑脚被动翻转
      —— 这是 SLIP 模型的前提，锁住髋角会强迫脚掌拖地、破坏动力学；
    * 腿只在**伸展段**（q̇ < 0）注入能量，压缩段交给被动弹簧储能。

能量伺服
--------
按「上一跳的起飞竖直速度」做增量修正::

    f_ex <- clamp(f_ex + k_e * (vz_des - vz_liftoff_prev) * m / stroke)

不直接闭环顶点高度，是因为触地冲击每跳要吃掉数焦耳，高度闭环会被冲击损耗
抵消、稳态顶点卡在目标以下（实测 0.454 m vs 目标 0.55 m）。改成闭环起飞速度后，
控制器自动把冲击损耗补偿掉。``vz_des = sqrt(2*g*(h_des - z_liftoff))``。

落点
----
Raibert 规则 ``x_offset = vx*T_st/2 + k_v*(vx - vx_des)``，脚落在身体前方
则减速、后方则加速，符号见 :func:`foot_placement_angle`。
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

import mujoco_hopper_slip_control.config as config


class Phase(Enum):
    """飞行 / 支撑。"""

    FLIGHT = "flight"
    STANCE = "stance"


@dataclass
class Command:
    """一步控制输出。"""

    leg_force: float          # 电机力，负值 = 蹬伸（对关节 +q 取负）
    hip_target: float         # 髋关节位置伺服目标角
    phase: Phase


@dataclass
class ControllerState:
    """外部可读的内部状态（写日志用）。"""

    phase: Phase = Phase.FLIGHT
    f_ex: float = config.FORCE_INIT
    q_compression: float = 0.03
    vz_liftoff_prev: float = 1.0
    z_liftoff_prev: float = 0.0
    apex_max: float = 0.0
    efficiency: float = 1.0
    foot_angle: float = 0.0


def required_liftoff_speed(h_des: float, z_liftoff: float, efficiency: float = 1.0,
                           g: float = config.GRAVITY) -> float:
    """给定离地高度与在线辨识的效率，反解需要的起飞竖直速度。

    纯弹道时 ``apex = z_lo + vz²/(2g)``；实测总有一部分能量耗在离地后的
    腿伸展与关节阻尼上，用 ``efficiency ∈ (0, 1]`` 折算::

        apex ≈ z_lo + efficiency * vz²/(2g)

    ``efficiency`` 由 :meth:`HopperController._on_touchdown` 每跳在线辨识，
    因此换目标高度、换弹簧刚度都不需要重标定常数。
    """
    eff = float(np.clip(efficiency, 0.4, 1.05))
    return float(np.sqrt(max(2.0 * g * max(h_des - z_liftoff, 0.0) / eff, 0.0)))


def foot_placement_angle(vx: float, vx_des: float,
                         k_v: Optional[float] = None,
                         t_st: Optional[float] = None,
                         leg_len: Optional[float] = None) -> float:
    """Raibert 落脚角。

    髋轴取 +y，``θ > 0`` 把脚放到身体 **-x** 侧（即身后）。落脚点相对身体
    的前向偏移为 ``x_offset = vx*T_st/2 + k_v*(vx - vx_des)``，脚在身后
    （x_offset < 0）时支撑期蹬地产生 +x 加速。

    增益在**函数体内**从 config 读取（而不是写成默认参数），否则 import 时
    就把值绑死了，运行时改 config 不生效。

    :return: 髋关节目标角，已按 ``RAIBERT_ANGLE_LIMIT`` 限幅。
    """
    k_v = config.RAIBERT_KV if k_v is None else k_v
    t_st = config.RAIBERT_T_ST if t_st is None else t_st
    leg_len = config.RAIBERT_LEG_LEN if leg_len is None else leg_len
    offset = vx * t_st / 2.0 + k_v * (vx - vx_des)
    ratio = float(np.clip(offset / leg_len, -1.0, 1.0))
    angle = float(np.arcsin(ratio))
    return float(np.clip(-angle, -config.RAIBERT_ANGLE_LIMIT, config.RAIBERT_ANGLE_LIMIT))


def desired_liftoff_speed(h_des: float, z_liftoff: float = config.Z_REST - 0.02,
                          g: float = config.GRAVITY) -> float:
    """把「顶点高度」折算成「起飞竖直速度」。"""
    return float(np.sqrt(max(2.0 * g * (h_des - z_liftoff), 0.0)))


class HopperController:
    """SLIP 弹跳 + Raibert 落点。纯函数式状态机，不依赖 MuJoCo。"""

    def __init__(self, target_height: float = config.TARGET_HEIGHT,
                 target_speed: float = 0.0,
                 energy_gain: float = config.ENERGY_GAIN,
                 mass: float = config.TOTAL_MASS,
                 debounce: int = config.DEBOUNCE_STEPS):
        self.target_height = float(target_height)
        self.target_speed = float(target_speed)
        self.energy_gain = float(np.clip(energy_gain, config.ENERGY_GAIN_MIN,
                                         config.ENERGY_GAIN_MAX))
        self.mass = float(mass)
        self.debounce = int(debounce)
        self.state = ControllerState()
        # 初值直接取目标起飞速度：否则能量伺服要从头爬升，前十几跳都处在瞬态
        self.state.vz_liftoff_prev = desired_liftoff_speed(self.target_height)
        self._cnt_in = 0
        self._cnt_out = 0
        self._vz_prev = 0.0

    # ------------------------------------------------------------------ 主循环
    def update(self, z: float, vz: float, vx: float, q: float, qd: float,
               hip_q: float, contact: bool) -> Command:
        """推进一个控制步。

        :param z: 躯干高度 (m)
        :param vz: 躯干竖直速度 (m/s)
        :param vx: 躯干水平速度 (m/s)
        :param q: 腿压缩量，``q > 0`` 表示压缩 (m)
        :param qd: 腿压缩速度 (m/s)
        :param hip_q: 当前髋角 (rad)
        :param contact: 原始触地标志（未去抖）
        """
        st = self.state
        self._cnt_in = self._cnt_in + 1 if contact else 0
        self._cnt_out = 0 if contact else self._cnt_out + 1

        # ---------------------------------------------------- 相位切换（去抖）
        if contact and st.phase is Phase.FLIGHT and self._cnt_in >= self.debounce:
            st.phase = Phase.STANCE
            self._on_touchdown()
        elif (not contact) and st.phase is Phase.STANCE and self._cnt_out >= self.debounce:
            st.phase = Phase.FLIGHT
            self._on_liftoff(vz, z)

        # 飞行相持续记录本跳最高点，供效率辨识
        if st.phase is Phase.FLIGHT:
            st.apex_max = max(st.apex_max, z)

        # ---------------------------------------------------- 顶点：一次算定落点
        if st.phase is Phase.FLIGHT and self._vz_prev > 0.0 >= vz:
            st.foot_angle = foot_placement_angle(vx, self.target_speed)
        self._vz_prev = vz

        # ---------------------------------------------------- 输出
        if st.phase is Phase.STANCE:
            st.q_compression = max(st.q_compression, q)
            # 只在伸展段注能，且伸展到接近离地时收手
            force = -st.f_ex if (qd < 0.0 and q > config.FORCE_STOP_Q) else 0.0
            # 髋：自由（SLIP 前提）或锁定在落脚角（保留侧向/纵向修正能力）
            hip = float(hip_q) if config.HIP_FREE_IN_STANCE else float(st.foot_angle)
            return Command(leg_force=float(force), hip_target=hip, phase=st.phase)

        hold = float(np.clip(config.LEG_HOLD_KP * (0.0 - q) - config.LEG_HOLD_KD * qd,
                             -config.LEG_HOLD_FORCE_LIMIT, config.LEG_HOLD_FORCE_LIMIT))
        return Command(leg_force=hold, hip_target=st.foot_angle, phase=st.phase)

    # ------------------------------------------------------------------ 事件
    def _on_touchdown(self) -> None:
        st = self.state
        # 在线辨识「离地后能量效率」：apex - z_lo ≈ η · vz²/(2g)
        if st.vz_liftoff_prev > 0.15:
            rise = max(st.apex_max - st.z_liftoff_prev, 0.0)
            eta = float(np.clip(rise * 2.0 * config.GRAVITY / st.vz_liftoff_prev ** 2,
                                config.EFFICIENCY_MIN, config.EFFICIENCY_MAX))
            st.efficiency = (1.0 - config.EFFICIENCY_SMOOTH) * st.efficiency + \
                config.EFFICIENCY_SMOOTH * eta
        vz_des = required_liftoff_speed(self.target_height, st.z_liftoff_prev,
                                        st.efficiency)
        stroke = max(st.q_compression - config.FORCE_STOP_Q, 0.02)
        delta = self.energy_gain * (vz_des - st.vz_liftoff_prev) * self.mass / stroke
        st.f_ex = float(np.clip(st.f_ex + delta, 0.0, config.FORCE_LIMIT))
        st.q_compression = 0.02

    def _on_liftoff(self, vz: float, z: float) -> None:
        st = self.state
        st.vz_liftoff_prev = float(vz)
        st.z_liftoff_prev = float(z)
        st.apex_max = float(z)

    # ------------------------------------------------------------------ 查询
    @property
    def phase(self) -> Phase:
        return self.state.phase

    @property
    def f_ex(self) -> float:
        return self.state.f_ex

    def reset(self) -> None:
        self.state = ControllerState()
        self._cnt_in = self._cnt_out = 0
        self._vz_prev = 0.0
