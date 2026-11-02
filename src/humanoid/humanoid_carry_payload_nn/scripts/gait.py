#!/usr/bin/env python3
"""gait.py —— 准静态双足步态生成器（搬运负载场景的"基座步态"）。

设计思路（与 docs 中的说明一致）：
  1. 一个完整步态周期分成"右腿摆动"和"左腿摆动"两个半步；
  2. 支撑腿（stance）髋关节角从 +A 线性转到 -A，脚踝反向补偿保持脚掌贴地，
     因此躯干在支撑脚上方前移，形成前进位移；
  3. 摆动腿（swing）髋关节从 -A 回到 +A，膝关节抬腿躲开地面，踝关节背屈；
  4. 骨盆左右平移与摆动相位对齐：摆动腿离地时，重心已移到另一侧支撑脚上方
     （hip_roll 平移 + ankle_roll 反向补偿保持脚掌水平）；
  5. 偏航通过"支撑腿髋 yaw 累积 + 摆动腿归零"实现原地/行进间转向。

本模块只做几何规划，不含任何动力学反馈；平衡补偿由 model.py 中的
PD 专家控制器 / 神经网络残差控制器叠加在关节目标角上。
"""

import math

#: 一个完整步态周期（秒）—— 包含左右各一步
CYCLE_TIME = 1.6
#: 单腿支撑相占比（其余为摆动相）
STANCE_RATIO = 0.60
#: 摆动抬腿时膝关节最大弯曲量（rad）——需保证脚掌离地
KNEE_LIFT = 0.78


class GaitGenerator:
    """相位式步态生成器。

    参数:
        stride_max: 髋关节摆动幅度上限（rad），对应最大步幅
        lift: 摆动腿抬腿高度（膝关节附加弯曲，rad）
        shift: 骨盆横移的等效关节角（rad）
        cycle_time: 完整步态周期（s）
    """

    def __init__(self, stride_max=0.22, lift=KNEE_LIFT, shift=0.0,
                 cycle_time=CYCLE_TIME, stance_ratio=STANCE_RATIO, knee_base=-0.04,
                 ankle_gain=1.0, hip_sign=1.0):
        self.stride_max = float(stride_max)
        self.lift = float(lift)
        self.shift = float(shift)
        self.cycle_time = float(cycle_time)
        self.stance_ratio = float(stance_ratio)
        self.knee_base = float(knee_base)
        self.ankle_gain = float(ankle_gain)
        self.hip_sign = float(hip_sign)
        self.phase = 0.0          # 归一化相位 [0, 1)
        self.yaw_l = 0.0          # 左腿支撑期累积的髋偏航（rad）
        self.yaw_r = 0.0

    def reset(self, phase=0.0):
        self.phase = float(phase) % 1.0
        self.yaw_l = 0.0
        self.yaw_r = 0.0

    # ------------------------------------------------------------------
    def advance(self, v_cmd, w_cmd, dt):
        """推进相位与转向累积量。

        参数:
            v_cmd: 前进速度指令（m/s，负值为后退）
            w_cmd: 偏航角速度指令（rad/s，正值为左转）
            dt: 采样步长（s）
        """
        speed = min(abs(v_cmd), 0.30)
        # 步长固定（由 stride_max 决定），步频与速度成正比。
        # 支撑期间"髋关节扫过的脚位移"必须正好等于"皮带走过的距离"
        # sweep = 2*L*sin(amp)，  sweep = v * stance_ratio * T
        # 保证不产生持续滑移与沿皮带方向的漂移。
        sweep = 2.0 * 0.72 * math.sin(min(abs(self.stride_max), 1.4))
        if speed < 1e-3:
            rate = 0.0
        else:
            rate = self.stance_ratio * speed / max(sweep, 0.05)
            rate = min(rate, 1.2 / self.cycle_time)
        self.phase = (self.phase + rate * dt) % 1.0

        # 转向：支撑腿髋偏航累积到 ±LIMIT，摆动腿归零重新落脚
        limit = 0.6 * math.pi  # ~0.6 rad
        for side in ("l", "r"):
            swing = self._in_swing(side)
            cur = getattr(self, "yaw_" + side)
            if swing:
                cur *= max(0.0, 1.0 - 4.0 * dt * 8.0)   # 摆动相快速回零
            else:
                cur += w_cmd * dt
                cur = max(-limit, min(limit, cur))
            setattr(self, "yaw_" + side, cur)

    # ------------------------------------------------------------------
    def _leg_phase(self, side):
        """某条腿在自身周期中的相位。"""
        return self.phase if side == "l" else (self.phase + 0.5) % 1.0

    def _in_swing(self, side):
        return self._leg_phase(side) >= self.stance_ratio

    # ------------------------------------------------------------------
    def targets(self, v_cmd):
        """返回 12 个腿部关节目标角（rad），键为关节名。

        步长固定（由 stride_max 决定），速度只改变步频（见 advance）；后退时相位轨迹反向。
        """
        amp = self.stride_max * (1.0 if abs(v_cmd) >= 0.02 else 0.0)
        direction = 1.0 if v_cmd >= 0.0 else -1.0

        out = {}
        for side in ("l", "r"):
            pl = self._leg_phase(side)
            if pl < self.stance_ratio:
                u = pl / self.stance_ratio
                # 髋 pitch 正向 = 腿向后摆，因此支撑腿从"后"转到"前"才能推动躯干前进
                hip = self.hip_sign * direction * amp * (2.0 * u - 1.0)
                knee = self.knee_base
                ankle = -self.ankle_gain * (hip + knee)
            else:
                u = (pl - self.stance_ratio) / (1.0 - self.stance_ratio)
                # 摆动腿从"前"回到"后"（相对躯干），同时抬膝越过地面
                hip = self.hip_sign * direction * amp * (1.0 - 2.0 * u)
                extra_lift = self.lift * math.sin(math.pi * u)
                knee = self.knee_base - extra_lift
                ankle = -(hip + knee) * 0.55 + 0.18 * math.sin(math.pi * u)
            out["hip_pitch_" + side] = hip
            out["knee_" + side] = knee
            out["ankle_pitch_" + side] = ankle
            out["hip_yaw_" + side] = getattr(self, "yaw_" + side)
            out["hip_roll_" + side] = 0.0
            out["ankle_roll_" + side] = 0.0

        # 骨盆横移：摆动腿离地时重心已移到支撑侧
        shift_norm = math.cos(2.0 * math.pi * (self.phase - 0.3))
        shift = self.shift * shift_norm * min(1.0, abs(v_cmd) / 0.10 + 0.35)
        for side in ("l", "r"):
            out["hip_roll_" + side] = -shift
            out["ankle_roll_" + side] = shift
        return out
