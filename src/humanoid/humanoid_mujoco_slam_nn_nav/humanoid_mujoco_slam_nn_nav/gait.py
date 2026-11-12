"""准静态双足步态生成器（世界坐标落脚点 + 重心转移）。

层次结构::

    (v, ω)
      └── Walker.update()  ──►  骨盆目标位姿 + 双脚目标位姿
              └── leg_ik()  ──►  12 个腿部关节角
                      └── MuJoCo position 伺服  ──►  力矩

关键设计
--------
1. **世界坐标落脚点**：脚落地后锚定在世界系，支撑相内绝不滑动，这样"走多远"
   与指令速度一致，不会像体坐标系方案那样靠打滑推进。
2. **重心转移梯形曲线**：单支撑开始之前就把骨盆移到支撑脚正上方（而不是等到
   单支撑中途才移），否则抬脚瞬间重心还在两脚之间，必然侧翻。
3. **闭环修正**：骨盆指令位置会缓慢跟随实测位置，避免打滑/外力造成的几何误差
   累积；同时用实测横滚/俯仰角做小增益补偿。

相位（一个周期 T，两条腿各一步）::

    phi_l = t/T,  phi_r = t/T + 0.5   （各自取模 1）

    |<---------- 支撑(phi<duty) ---------->|<- 摆动 ->|
    0                                    duty         1
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

from . import kinematics as K

SIDES = ("l", "r")


def _smoothstep(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


@dataclass
class GaitParams:
    """步态参数（可通过 ROS 参数服务器在线调整）。"""

    period: float = 1.10          # 一个完整周期（左右各一步），s
    duty: float = 0.66            # 单腿支撑相占比（>0.5 才有双支撑）
    step_height: float = 0.045    # 摆动脚抬升高度，m
    side_shift: float = 0.075     # 重心横向转移幅度，m
    pelvis_height: float = 0.805  # 骨盆目标高度，m
    bob: float = 0.008            # 骨盆上下起伏，m
    v_max: float = 0.40           # 最大前进速度，m/s
    w_max: float = 0.50           # 最大转向角速度，rad/s
    stride_max: float = 0.18      # 单步最大步幅，m
    foot_spread: float = 0.085    # 双脚横向间距（半宽），m
    accel: float = 1.2            # 速度指令一阶滤波系数（1/s）
    sync_gain: float = 1.5        # 指令位姿向实测位姿收敛的增益（1/s）
    roll_gain: float = 0.25       # 横滚反馈增益
    pitch_gain: float = 0.25      # 俯仰反馈增益
    shift_profile: str = "sine"   # "sine"（单支撑中途最大）或 "trapezoid"
    land_ahead: float = 0.55      # 落脚点在骨盆前方的比例（相对步幅）
    ankle_kp: float = 0.9         # 踝姿态反馈：比例增益（rad/rad）
    ankle_kd: float = 0.10        # 踝姿态反馈：微分增益（rad/(rad/s)）
    pelvis_mode: str = "feet"     # "feet"=骨盆跟随双脚中点（准静态）；"path"=按指令路径
    capture_gain: float = 0.35    # 横向"捕获点"增益：落脚点随侧向速度前移（s）
    capture_max: float = 0.10     # 捕获点横向修正上限，m
    com_kp: float = 0.6           # 重心位置误差 -> 骨盆修正的比例增益
    com_kd: float = 0.06          # 重心速度阻尼（s）
    com_max: float = 0.05         # 单周期骨盆修正上限，m
    lead_gain: float = 0.0        # 骨盆相对双脚的前置量 = lead_gain * v（s）


@dataclass
class FootState:
    x: float
    y: float
    yaw: float = 0.0
    swing: bool = False
    sw_from: Tuple[float, float] = (0.0, 0.0)
    sw_to: Tuple[float, float] = (0.0, 0.0)
    yaw_from: float = 0.0
    yaw_to: float = 0.0


class Walker:
    """把 (v, ω) 速度指令转成 12 个腿部关节目标角。"""

    def __init__(self, params: GaitParams | None = None, x: float = 0.0, y: float = 0.0,
                 yaw: float = 0.0):
        self.p = params or GaitParams()
        self.reset(x, y, yaw)
        self.v_cmd = 0.0
        self.w_cmd = 0.0

    # ------------------------------------------------------------------
    def reset(self, x: float, y: float, yaw: float):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.t = 0.0
        self._phase = 0.0
        self.v = 0.0
        self.w = 0.0
        self.roll_bias = 0.0
        self.pitch_bias = 0.0
        self.yaw_err = 0.0
        self.feet = {
            "l": FootState(x, y + self.p.foot_spread, yaw),
            "r": FootState(x, y - self.p.foot_spread, yaw),
        }
        self._prev_phi = {"l": 0.0, "r": 0.5}
        self.vy = 0.0
        self.capture_y = 0.0
        self.actual_feet = None

    def command(self, v: float, w: float):
        self.v_cmd = float(np.clip(v, -0.8 * self.p.v_max, self.p.v_max))
        self.w_cmd = float(np.clip(w, -self.p.w_max, self.p.w_max))

    # ------------------------------------------------------------------
    @staticmethod
    def _leg_phase(side: str, phase: float) -> float:
        return (phase + (0.5 if side == "r" else 0.0)) % 1.0

    def _shift(self, phase: float) -> float:
        """重心横向转移。

        以 ``duty`` 为单腿支撑占比，各窗口为::

            [0, duty-0.5]   双支撑，重心 -A -> +A
            [duty-0.5, 0.5] 左腿单支撑          +A
            [0.5, duty]     双支撑，重心 +A -> -A
            [duty, 1]       右腿单支撑          -A
        """
        p, d = self.p, self.p.duty
        if p.shift_profile == "sine":
            h = (phase % 0.5) / 0.5
            stance = "l" if (phase % 1.0) < 0.5 else "r"
            return p.side_shift * math.sin(math.pi * h) * (1.0 if stance == "l" else -1.0)
        if phase < d - 0.5:
            a = -1.0 + 2.0 * _smoothstep(phase / max(d - 0.5, 1e-6))
        elif phase < 0.5:
            a = 1.0
        elif phase < d:
            a = 1.0 - 2.0 * _smoothstep((phase - 0.5) / max(d - 0.5, 1e-6))
        else:
            a = -1.0
        return p.side_shift * a

    def _bob(self, phase: float) -> float:
        return -self.p.bob * 0.5 * (1.0 - math.cos(4 * math.pi * phase))

    # ------------------------------------------------------------------
    def update(self, dt: float, actual_pos=None, actual_yaw: float | None = None,
               roll: float = 0.0, pitch: float = 0.0,
               roll_rate: float = 0.0, pitch_rate: float = 0.0,
               vel=None, feet_xy: dict | None = None) -> Dict[str, float]:
        p = self.p
        self.actual_feet = feet_xy
        alpha = min(1.0, p.accel * dt)
        self.v += (self.v_cmd - self.v) * alpha
        self.w += (self.w_cmd - self.w) * alpha

        prev_phase = self._phase
        self.t += dt
        self._phase = (self.t / p.period) % 1.0
        phase = self._phase

        # 侧向速度（体坐标 y 方向），用于"落脚点捕获"横向平衡
        self.vy = 0.0
        if vel is not None:
            self.vy = -math.sin(self.yaw) * float(vel[0]) + math.cos(self.yaw) * float(vel[1])
        self.capture_y = float(np.clip(p.capture_gain * self.vy, -p.capture_max, p.capture_max))

        # 指令位姿积分（转向时走圆弧）
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        # 闭环同步：指令位姿缓慢贴向实测位姿，防止打滑/外力累积误差
        if actual_pos is not None:
            k = min(1.0, p.sync_gain * dt)
            self.x += (float(actual_pos[0]) - self.x) * k
            self.y += (float(actual_pos[1]) - self.y) * k
        if actual_yaw is not None:
            self.yaw_err = math.atan2(math.sin(self.yaw - actual_yaw),
                                      math.cos(self.yaw - actual_yaw))
            self.yaw += -self.yaw_err * min(1.0, 0.8 * dt)
            self.yaw_err = math.atan2(math.sin(self.yaw - actual_yaw),
                                      math.cos(self.yaw - actual_yaw))

        # 姿态反馈（小增益，防止正反馈振荡）
        self.roll_bias = float(np.clip(self.roll_bias + p.roll_gain * roll * dt, -0.03, 0.03))
        self.pitch_bias = float(np.clip(self.pitch_bias + p.pitch_gain * pitch * dt, -0.03, 0.03))

        # 半步切换 -> 重新规划落脚点
        if int(prev_phase * 2) != int(phase * 2) or self.t <= dt * 1.5:
            self._plan_swing(phase)

        self._land_finished(phase)

        shift = self._shift(phase) + self.roll_bias

        # 先算双脚目标（世界坐标），再决定骨盆目标
        q: Dict[str, float] = {}
        targets = {}
        for s in SIDES:
            f = self.feet[s]
            targets[s] = self._foot_target(s, phase, f)

        if p.pelvis_mode == "feet":
            # 准静态模式：骨盆目标 = 双脚目标点中点 + 重心偏移 + CoM 跟踪反馈。
            # 中点保证骨盆不会"跑"到脚前面；CoM 反馈用实测位移/速度把身体拉回
            # 期望重心位置；lead 项相对双脚把身体往前"压"，提供前进驱动力。
            mx = 0.5 * (targets["l"][0] + targets["r"][0])
            my = 0.5 * (targets["l"][1] + targets["r"][1])
            lead = p.lead_gain * self.v
            px = mx - math.sin(self.yaw) * shift + math.cos(self.yaw) * lead
            py = my + math.cos(self.yaw) * shift + math.sin(self.yaw) * lead
            if actual_pos is not None:
                ex = px - float(actual_pos[0])
                ey = py - float(actual_pos[1])
                vx = 0.0 if vel is None else float(vel[0])
                vyw = 0.0 if vel is None else float(vel[1])
                # 只做"修正量"，不改变整体目标，避免积分漂移
                cx = p.com_kp * ex - p.com_kd * vx
                cy = p.com_kp * ey - p.com_kd * vyw
                cx = float(np.clip(cx, -p.com_max, p.com_max))
                cy = float(np.clip(cy, -p.com_max, p.com_max))
                px += cx
                py += cy
            pz = p.pelvis_height + self._bob(phase)
            self.x, self.y = mx, my
        else:
            px = self.x - math.sin(self.yaw) * shift + math.cos(self.yaw) * self.pitch_bias
            py = self.y + math.cos(self.yaw) * shift + math.sin(self.yaw) * self.pitch_bias
            pz = p.pelvis_height + self._bob(phase)
        pelvis = (px, py, pz)

        for s in SIDES:
            fx, fy, fz, fyaw = targets[s]
            ankle = np.array([fx, fy, K.stand_ankle_z() + fz])
            qi = K.leg_ik(s, pelvis, self.yaw, ankle, fyaw)
            q.update({f"{k}_{s}": v for k, v in qi.items()})

        sw = math.sin(2 * math.pi * phase)
        q["shoulder_pitch_l"] = 0.12 + 0.20 * sw
        q["shoulder_pitch_r"] = -0.12 + 0.20 * sw
        q["elbow_pitch_l"] = -0.25 - 0.08 * abs(sw)
        q["elbow_pitch_r"] = -0.25 - 0.08 * abs(sw)
        q["waist_yaw"] = float(np.clip(-0.10 * self.w, -0.35, 0.35))

        # 踝关节姿态反馈（ankle strategy）：躯干前倾时蹬地，侧倾时用踝侧摆回正
        corr_p = float(np.clip(-p.ankle_kp * pitch - p.ankle_kd * pitch_rate, -0.30, 0.30))
        corr_r = float(np.clip(p.ankle_kp * roll + p.ankle_kd * roll_rate, -0.30, 0.30))
        for s in SIDES:
            q[f"ankle_pitch_{s}"] = q.get(f"ankle_pitch_{s}", 0.0) + corr_p
            q[f"ankle_roll_{s}"] = q.get(f"ankle_roll_{s}", 0.0) + corr_r

        self._last_pelvis = pelvis
        return q

    # ------------------------------------------------------------------
    def _plan_swing(self, phase: float):
        """为即将摆动的腿计算落脚点（世界坐标）。"""
        p = self.p
        stance = "l" if (phase % 1.0) < 0.5 else "r"
        swing = "r" if stance == "l" else "l"
        f = self.feet[swing]
        f.sw_from = (f.x, f.y)
        f.yaw_from = f.yaw

        if p.pelvis_mode == "feet":
            # 准静态：落脚点 = 支撑脚 + 一个步距（沿朝向），横向保持 2*spread
            st = self.feet[stance]
            step = float(np.clip(self.v * p.period, -2 * p.stride_max, 2 * p.stride_max))
            side = 2.0 * p.foot_spread if swing == "l" else -2.0 * p.foot_spread
            base_y = p.foot_spread if swing == "l" else -p.foot_spread
            # 支撑脚在另一侧，因此横向偏移为 ±2*spread
            dy = (base_y - (-p.foot_spread if swing == "l" else p.foot_spread))
            f.sw_to = (st.x + math.cos(self.yaw) * step - math.sin(self.yaw) * dy,
                       st.y + math.sin(self.yaw) * step + math.cos(self.yaw) * dy)
            f.sw_to = (f.sw_to[0] - math.sin(self.yaw) * self.capture_y,
                       f.sw_to[1] + math.cos(self.yaw) * self.capture_y)
            f.yaw_to = self.yaw
            f.swing = True
            del side
            return

        swing_time = (1.0 - p.duty) * p.period
        # 以摆动结束时的预测位姿为基准
        t_end = swing_time * 0.5 + 0.1 * p.period
        v, w = self.v, self.w
        if abs(w) > 1e-3:
            r = v / w
            nx = self.x + r * (math.sin(self.yaw + w * t_end) - math.sin(self.yaw))
            ny = self.y - r * (math.cos(self.yaw + w * t_end) - math.cos(self.yaw))
        else:
            nx = self.x + v * math.cos(self.yaw) * t_end
            ny = self.y + v * math.sin(self.yaw) * t_end
        nyaw = self.yaw + w * t_end

        stride = float(np.clip(abs(v) * p.period * 0.5, 0.0, p.stride_max))
        ahead = stride * p.land_ahead
        side = p.foot_spread if swing == "l" else -p.foot_spread
        f.sw_to = (nx + math.cos(nyaw) * ahead - math.sin(nyaw) * side,
                   ny + math.sin(nyaw) * ahead + math.cos(nyaw) * side)
        f.yaw_to = nyaw
        f.swing = True

    def _land_finished(self, phase: float):
        """摆动结束（相位回绕）后，把脚锚定在落点上。

        注意必须"回绕"才算落地：``_leg_phase`` 在 [0, duty) 区间是支撑相，
        如果一进入支撑相就落锚，正在支撑的那条腿会被瞬间拉向前方落脚点，
        反而把躯干往回蹬（这是早期版本原地后退的直接原因）。
        """
        for s in SIDES:
            f = self.feet[s]
            phi = self._leg_phase(s, phase)
            prev = self._prev_phi.get(s, phi)
            if f.swing and prev > 0.85 and phi < 0.15:
                # 用"实测踝位置"落锚，而不是指令落点：打滑时指令落点会越飘越远，
                # 实测落锚才能保证后续 IK 始终以真实脚位为基准。
                if self.actual_feet and s in self.actual_feet:
                    f.x, f.y = self.actual_feet[s]
                else:
                    f.x, f.y = f.sw_to
                f.yaw = f.yaw_to
                f.swing = False
            self._prev_phi[s] = phi

    def _foot_target(self, side: str, phase: float, f: FootState):
        """返回 (世界 x, 世界 y, 抬升高度 z, 脚掌 yaw)。"""
        p = self.p
        phi = self._leg_phase(side, phase)
        if phi < p.duty:
            # 支撑相：锚点固定
            return f.x, f.y, 0.0, f.yaw
        u = (phi - p.duty) / max(1.0 - p.duty, 1e-6)
        sm = _smoothstep(u)
        x0, y0 = f.sw_from
        x1, y1 = f.sw_to
        z = p.step_height * math.sin(math.pi * u)
        dyaw = (f.yaw_to - f.yaw_from + math.pi) % (2 * math.pi) - math.pi
        return x0 + (x1 - x0) * sm, y0 + (y1 - y0) * sm, z, f.yaw_from + dyaw * sm


def stand_targets(x: float, y: float, yaw: float, knee: float = 0.30):
    """站立关节目标（停止状态使用）。"""
    _, q = K.default_stand((x, y), yaw, knee)
    return q
