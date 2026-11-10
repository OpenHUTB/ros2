"""底层控制：管道跟踪 PD + 纯追踪 + 基于深度图的安全减速。

管道跟踪是级联结构：外环用感知给的 (dx_err, dtheta) 算期望角速度，
内环压速度。纯追踪用于沿地图路径（感知丢失时靠建图兜底）的路段。
"""

import math

import numpy as np

from . import params as P
from .kinematics import wrap

KP_TH = 1.9          # 艏向误差增益
KP_LAT = 0.85        # 横向误差增益（通过期望艏向注入）
KI_LAT = 0.05        # 横向误差积分增益：海流是里程计看不见的常值扰动，
                     # 纯 PD 会有正比于流速的稳态偏差，积分项把它吃掉
I_LAT_MAX = 0.50     # 积分限幅（弧度）：足够抵消 CURRENT_MAX 的海流
V_GAIN_LAT = 0.55    # 横向误差对速度的压制
V_GAIN_TH = 0.50     # 艏向误差对速度的压制
SPIN_ENTER = 0.70    # 进入原地回转的期望艏向阈值
SPIN_EXIT = 0.45     # 退出回转的阈值（迟滞，避免在边界来回抖动）


class PipeFollower:
    """管道跟踪级联控制器（有状态：横向积分 + 回转迟滞）。

    级联结构：期望艏向 = dtheta + atan(k·dx_err) + 积分项；
    期望艏向大（折返 U 转弯）时原地回转 —— v=0 时 ROV 不平移，
    带速度的大半径转弯会扫到路边的应答器甚至管道。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.i_lat = 0.0
        self.spinning = False
        self.turn_dir = 0.0

    def begin_turnaround(self, direction):
        """折返机动（往返巡检的 180° 掉头）。

        direction=+1 原地左回转（CCW），-1 右回转。方向必须指向背离
        管道的一侧：掉头时 ROV 还带着 ~0.5 m/s 的残余推力，如果朝
        管道一侧转，头一两秒的弧线会直接扫进管道。压速靠 v=0 指令 +
        推进器一阶滞后自然完成，回转全程无位移风险。
        """
        self.turn_dir = float(direction)

    def step(self, dx_err, dtheta, max_v=None):
        """(任务系横向误差, 艏向误差) → (u, w)。

        dx_err 在**任务系**下定义：>0 表示还要向巡检方向的左法向挪。
        去程与任务系一致；返程艏向反 180°，同一个物理偏差符号相反，
        必须由调用方（策略层，知道巡检方向）先换算 —— 横向项符号
        不换算是早期版本返程撞管、漂出世界的直接原因。
        """
        max_v = P.MAX_V if max_v is None else max_v
        # 折返机动优先：不感知、不安全偏转，转到艏向误差收小为止
        if self.turn_dir:
            if abs(dtheta) < SPIN_EXIT:
                self.turn_dir = 0.0
            else:
                return 0.0, float(np.clip(self.turn_dir * P.MAX_W,
                                          -P.MAX_W, P.MAX_W))
        desired = wrap(dtheta + math.atan2(KP_LAT * dx_err, 1.4))
        # 回转迟滞：进入阈值高、退出阈值低，防止在边界处一拍回转一拍 PD
        if abs(desired) > SPIN_ENTER:
            self.spinning = True
        elif self.spinning and abs(desired) < SPIN_EXIT:
            self.spinning = False
        if self.spinning:
            return 0.0, float(np.clip(2.0 * desired, -P.MAX_W, P.MAX_W))
        # 直线段才积分：回转/大偏差期间冻结，防止积分饱和
        self.i_lat = float(np.clip(
            self.i_lat + KI_LAT * P.DT * dx_err, -I_LAT_MAX, I_LAT_MAX))
        desired = wrap(desired + self.i_lat)
        w = float(np.clip(KP_TH * desired, -P.MAX_W, P.MAX_W))
        v = max_v * float(np.clip(
            1.0 - V_GAIN_LAT * abs(dx_err) - V_GAIN_TH * abs(desired), 0.25, 1.0))
        return v, w


def follow_control(dx_err, dtheta, max_v=None):
    """无状态便捷版（无积分项、无迟滞），供测试与对照实验使用。"""
    max_v = P.MAX_V if max_v is None else max_v
    desired = dtheta + math.atan2(KP_LAT * dx_err, 1.4)
    desired = wrap(desired)
    if abs(desired) > SPIN_ENTER:
        return 0.0, float(np.clip(2.0 * desired, -P.MAX_W, P.MAX_W))
    w = float(np.clip(KP_TH * desired, -P.MAX_W, P.MAX_W))
    v = max_v * float(np.clip(
        1.0 - V_GAIN_LAT * abs(dx_err) - V_GAIN_TH * abs(desired), 0.25, 1.0))
    return v, w


def pure_pursuit(pose, path, lookahead=None):
    """返回 (v, w)。路径为空时原地停住。"""
    lookahead = P.LOOKAHEAD if lookahead is None else lookahead
    if not path:
        return 0.0, 0.0

    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    pts = np.asarray(path, dtype=np.float64)

    d2 = (pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2
    i0 = int(np.argmin(d2))

    target = pts[-1]
    on_final = True
    for i in range(i0, len(pts)):
        if math.hypot(pts[i, 0] - x, pts[i, 1] - y) >= lookahead:
            target = pts[i]
            on_final = (i == len(pts) - 1)
            break

    dx = target[0] - x
    dy = target[1] - y
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return 0.0, 0.0

    alpha = wrap(math.atan2(dy, dx) - th)

    # 终点收敛段：纯追踪在前视距离之外会绕着终点转圈，这里改成
    # 角度闭环 + 随距离衰减的速度，能稳稳停在 GOAL_TOL 之内。
    if on_final and L < 0.90:
        w = float(np.clip(2.4 * alpha, -P.MAX_W, P.MAX_W))
        v = P.MAX_V * float(np.clip((L - 0.16) / 0.55, 0.0, 1.0))
        return v, w
    kappa = 2.0 * math.sin(alpha) / max(L, 0.15)
    v = P.MAX_V * float(np.clip(1.0 - 0.55 * abs(alpha) / (math.pi / 2), 0.30, 1.0))
    w = float(np.clip(v * kappa, -P.MAX_W, P.MAX_W))
    return v, w


def safety_from_depth(depth, v, w, near=0.55, stop=0.32, gain=0.40):
    """用一帧深度做安全减速：应答器太近就减速偏转，贴脸就停。

    巡检路径与应答器的设计间距是 0.9 m，正常跟踪不该触发；
    这是横向误差变大时的最后一道保险。
    """
    if depth is None:
        return v, w
    H, W = depth.shape
    band = depth[H // 2, :]
    c = W // 2
    half = max(3, W // 10)

    front = float(np.percentile(band[c - half:c + half + 1], 15.0))
    left = float(np.median(band[c + half: c + 3 * half + 1]))
    right = float(np.median(band[c - 3 * half: c - half + 1]))

    if front > near:
        return v, w

    scale = float(np.clip((front - stop) / (near - stop), 0.0, 1.0))
    if left > right + 0.15:
        bias = gain * (1.0 - scale)
    elif right > left + 0.15:
        bias = -gain * (1.0 - scale)
    else:
        bias = 0.75 * gain * (1.0 - scale) * (1.0 if left >= right else -1.0)
    w = float(np.clip(w + bias, -P.MAX_W, P.MAX_W))
    return v * scale, w


def goal_reached(pose, goal, tol=None):
    tol = P.GOAL_TOL if tol is None else tol
    return math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < tol
