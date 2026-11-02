# -*- coding: utf-8 -*-
"""步态生成与关节逆解。

两件事：
1. **足端轨迹** —— 每只脚按相位在「落脚点 → 下一个落脚点」之间画一条抬腿曲线，
   支撑相固定在落脚点上不动；
2. **解析 IK** —— 由髋到踝的向量直接解出 hip / knee 两个角度（两连杆闭式解），
   踝关节角让脚底保持水平。

关节方向约定（MuJoCo，轴均为 +y）：hip 正角度让腿向 -x 摆，knee 正角度为屈膝，
ankle 正角度让脚尖上翘。IK 已把这个符号吸收掉，调用方只给足端世界坐标。

本模块不依赖 MuJoCo，纯数学，便于单元测试。
"""

import math

import humanoid_stair_climb.config as config

# 两只脚的摆动窗口（在周期内的相位区间）
SWING_WINDOW = {"l": (0.0, config.SWING_RATIO),
                "r": (0.5, 0.5 + config.SWING_RATIO)}


# ---------------------------------------------------------------- 地形
def ground_height(x):
    """给定 x 返回台阶顶面高度（平地 0，逐级 0.12，平台 0.72）。"""
    if x < 0.0:
        return 0.0
    k = int(math.floor(x / config.STEP_DEPTH)) + 1
    if k >= config.STEP_COUNT:
        return config.LANDING_TOP
    return k * config.STEP_HEIGHT


def step_index(x):
    """x 所在的台阶级号：0 = 平地，1..STEP_COUNT = 台阶，STEP_COUNT+1 = 平台。"""
    if x < 0.0:
        return 0
    k = int(math.floor(x / config.STEP_DEPTH)) + 1
    return min(k, config.STEP_COUNT + 1)


def smootherstep(s):
    """五次平滑插值，两端一阶、二阶导都为 0。"""
    s = min(1.0, max(0.0, s))
    return s * s * s * (s * (s * 6.0 - 15.0) + 10.0)


# ---------------------------------------------------------------- 步态
class StairGait:
    """双腿交替步态：每只脚每周期前进一步，两脚摆动窗口错开半个周期。"""

    def __init__(self, cycle_time=None, swing_ratio=None, stride=None,
                 lift_height=None, start_l=None, start_r=None):
        self.cycle_time = float(cycle_time or config.CYCLE_TIME)
        self.swing_ratio = float(swing_ratio or config.SWING_RATIO)
        self.stride = float(stride or config.STRIDE)
        self.lift = float(lift_height or config.LIFT_HEIGHT)
        self.start_l = config.FOOT_START_L if start_l is None else float(start_l)
        self.start_r = config.FOOT_START_R if start_r is None else float(start_r)

    # ------------------------------------------------------------
    def base_x(self, t):
        """基座前进位置：匀速，速度 = 步幅 / 周期。"""
        return config.BASE_X0 + config.BASE_SPEED * t

    def _phase(self, t):
        """返回 (k, s)：k = 已完成周期数，s = 周期内相位 [0, 1)。"""
        ph = t / self.cycle_time
        k = int(math.floor(ph))
        return k, ph - k

    def _start(self, side):
        return self.start_l if side == "l" else self.start_r

    def foot_state(self, t, side):
        """返回该脚的世界坐标 (x, z) 与是否处于摆动相。"""
        k, s = self._phase(t)
        start = self._start(side)
        a, b = SWING_WINDOW[side]
        x_before = start + k * self.stride
        x_after = x_before + self.stride

        if a <= s < b:
            u = (s - a) / (b - a)
            x, z = self._swing_profile(x_before, x_after,
                                       ground_height(x_before), ground_height(x_after), u)
            return x, z, True
        if s < a:
            x = x_before
        else:
            x = x_after
        return x, ground_height(x), False

    def _swing_profile(self, x0, x1, z0, z1, u):
        """摆动相足端轨迹：先原地抬到安全高度 → 水平移动 → 落到目标。

        为什么不是「线性上升 + 正弦拱」：脚是 0.24 m 长的盒子，前端比踝超前
        0.18 m。若用正弦拱，踝还在立面之前时脚前端已经伸进台阶里，脚底高度
        又不够，就会穿模。分段轨迹保证**水平移动全程脚底都在安全高度**。
        """
        z_safe = max(z0, z1) + self.lift
        if u < 0.25:                                   # 原地抬升
            s = u / 0.25
            return x0, z0 + (z_safe - z0) * smootherstep(s)
        if u < 0.75:                                   # 水平移动
            s = (u - 0.25) / 0.5
            return x0 + (x1 - x0) * smootherstep(s), z_safe
        s = (u - 0.75) / 0.25                          # 落下
        return x1, z_safe + (z1 - z_safe) * smootherstep(s)

    def landing_height(self, t, side):
        """该脚在当前周期结束时要落到的台阶高度。"""
        k, _ = self._phase(t)
        x_after = self._start(side) + (k + 1) * self.stride
        return ground_height(x_after)

    def support_height(self, t):
        """重心参考高度：取两只脚里**最低**的那个目标高度。

        支撑脚用当前所在台阶；摆动脚用它即将落到的台阶——必须在落下之前就
        开始降重心，否则摆动脚刚接触低处时髋到踝距离会超过腿长（实测拍到
        r=0.905 > 0.90，IK 被 clamp，脚被拖进台阶立面）。

        取「最低」而不是「最高」：上台阶时重心跟着低处那只脚走，高处的脚
        多屈一点膝，全程不超腿长。
        """
        heights = []
        for side in ("l", "r"):
            _, z, swing = self.foot_state(t, side)
            heights.append(self.landing_height(t, side) if swing else z)
        return min(heights)

    # ------------------------------------------------------------
    @staticmethod
    def leg_ik(dx, dz):
        """两连杆解析 IK。

        dx, dz: 踝关节相对髋关节的位置（dz 为负，腿在下方）。
        返回 (hip, knee) 弧度。
        """
        r = math.hypot(dx, dz)
        r = min(config.LEG_MAX, max(config.LEG_MIN, r))

        cos_knee = (config.THIGH ** 2 + config.SHANK ** 2 - r ** 2) / (
            2.0 * config.THIGH * config.SHANK)
        cos_knee = min(1.0, max(-1.0, cos_knee))
        knee = math.pi - math.acos(cos_knee)

        phi = math.atan2(dx, -dz)
        cos_a = (config.THIGH ** 2 + r ** 2 - config.SHANK ** 2) / (
            2.0 * config.THIGH * r)
        cos_a = min(1.0, max(-1.0, cos_a))
        alpha = math.acos(cos_a)
        hip = -(phi + alpha)
        return hip, knee

    @staticmethod
    def ankle_angle(hip, knee):
        """让脚底保持水平：踝角抵消髋 + 膝的累计转角。"""
        return -(hip + knee)

    def joint_targets(self, t, base_x, base_z):
        """返回 6 个关节角：(hip_l, knee_l, ankle_l, hip_r, knee_r, ankle_r)。"""
        out = []
        for side in ("l", "r"):
            fx, fz_ground, _ = self.foot_state(t, side)
            ankle_z = fz_ground + config.FOOT_DROP      # 踝高 = 脚底 + 0.04
            hip, knee = self.leg_ik(fx - base_x, ankle_z - (base_z + config.HIP_Z))
            out.extend([hip, knee, self.ankle_angle(hip, knee)])
        return out
