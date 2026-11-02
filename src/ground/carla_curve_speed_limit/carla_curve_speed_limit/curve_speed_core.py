"""弯道自适应限速（Curve Speed Limit，CSL）决策核心。

纯离线逻辑：不依赖 ROS 与 CARLA，可独立单元测试。

物理模型：

  1) 路点曲率：自车前方路点序列 (x,y)，用相邻三点外接圆法计算带符号
     曲率 kappa（左转为正、右转为负），|kappa| 越大弯越急；
  2) 弯道限速：v_max = sqrt(a_lat_max / |kappa|)——由横向加速度上限
     a_lat_max（默认 2.5 m/s²，舒适阈值）反解安全过弯速度；
  3) 预瞄取严：沿前瞻窗口（默认 30 m）逐路点计算限速，取最小值作为
     当前目标速度——入弯前提前减速，而非进弯才刹。

决策：

  * 滞回状态机 cruise → slowdown：目标速度低于巡航速度超过 slow_enter
    （0.5 m/s）进入减速；差距收窄到 slow_exit（0.2 m/s）以内才恢复，
    消除临界抖动；
  * 斜坡平滑：输出速度指令向目标速度以 decel_max/accel_max 限幅逼近，
    弯道限速阶梯变化被平滑为可执行的加减速；
  * v_min 下限：弯道再急也不低于最低速度（默认 1.5 m/s），避免弯中停车。
"""

from dataclasses import dataclass

import numpy as np

CRUISE = "cruise"
SLOWDOWN = "slowdown"


def signed_curvature(p0, p1, p2):
    """三点外接圆曲率（1/m），左转为正。

    p0/p1/p2: (x, y) 序列上相邻三点。共线时曲率为 0。
    """
    ax, ay = p1[0] - p0[0], p1[1] - p0[1]
    bx, by = p2[0] - p1[0], p2[1] - p1[1]
    cross = ax * by - ay * bx
    denom = np.hypot(ax, ay) * np.hypot(bx, by) * np.hypot(p2[0] - p0[0],
                                                           p2[1] - p0[1])
    if denom == 0:
        return 0.0
    return float(2.0 * cross / denom)


def waypoint_curvatures(points):
    """路点序列 (N,2) → 各点曲率 (N,)（首尾点复制邻值）。

    输入应按行驶方向排序（自车前方 → 远处）。
    """
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    if n < 3:
        return np.zeros(n)
    k = np.zeros(n)
    for i in range(1, n - 1):
        k[i] = signed_curvature(pts[i - 1], pts[i], pts[i + 1])
    k[0], k[-1] = k[1], k[-2]
    return k


def curve_speed_limit(kappa, a_lat_max=2.5, valid_max=30.0):
    """曲率 → 安全过弯速度 v = sqrt(a_lat_max / |kappa|)。

    直道（|kappa| < 1e-6）返回 valid_max。
    """
    ak = abs(kappa)
    if ak < 1e-6:
        return float(valid_max)
    return float(min(np.sqrt(a_lat_max / ak), valid_max))


def target_speed_along(points, a_lat_max=2.5, valid_max=30.0):
    """前瞻路点 → 预瞄目标速度（全程最严限速）。"""
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return float(valid_max)
    k = waypoint_curvatures(pts)
    return min(curve_speed_limit(kk, a_lat_max, valid_max) for kk in k)


@dataclass
class CSLParams:
    """CSL 决策参数（与 config/csl_params.yaml 一一对应）。"""
    cruise_speed: float = 8.0   # 巡航速度（m/s）
    a_lat_max: float = 2.5      # 横向加速度上限（m/s²）
    look_ahead: float = 30.0    # 预瞄距离（m，决定取路点范围）
    v_min: float = 1.5          # 弯道最低速度（m/s），防弯中停车
    decel_max: float = 2.0      # 减速度限幅（m/s²）
    accel_max: float = 1.5      # 加速度限幅（m/s²）
    slow_enter: float = 0.5     # 目标低于巡航该差值 → slowdown
    slow_exit: float = 0.2      # 差值收窄到该值以内 → 恢复 cruise（滞回）
    valid_max: float = 30.0


def csl_decide(state, target_speed, p=None):
    """单步滞回决策（纯函数）：cruise/slowdown → (new_state, event)。

    event 仅在 new_state 与传入 state 不同（真实跃迁）时非空：
    slowdown_warn / cruise_resume。
    """
    p = p or CSLParams()
    gap = p.cruise_speed - target_speed
    old = state
    if state == CRUISE:
        state = SLOWDOWN if gap > p.slow_enter else CRUISE
    elif state == SLOWDOWN:
        state = CRUISE if gap < p.slow_exit else SLOWDOWN
    else:
        raise ValueError(f"非法状态 {state!r}，应为 cruise/slowdown")
    event = ""
    if state != old:
        event = "slowdown_warn" if state == SLOWDOWN else "cruise_resume"
    return state, event


def ramp_speed(commanded, target, dt, p=None):
    """斜坡限速：commanded 向 target 以 decel/accel 限幅逼近（纯函数）。"""
    p = p or CSLParams()
    if target < commanded:  # 减速
        step = p.decel_max * dt
        return float(max(target, commanded - step))
    step = p.accel_max * dt
    return float(min(target, commanded + step))


class CSLController:
    """CSL 有状态封装：滞回状态机 + 速度斜坡。

    tick() 每周期喂入前瞻路点（或已算好的目标速度），返回
    {state, target_speed, commanded_speed, event}。
    """

    def __init__(self, params=None):
        self.p = params or CSLParams()
        self._state = CRUISE
        self._commanded = self.p.cruise_speed

    def target_from_points(self, points, dt=0.05):
        """前瞻路点 → 目标速度 = min(巡航速度, 预瞄限速)，再受 v_min 兜底。

        限速是上限不是设定值：直道限速=valid_max 时目标回落到巡航速度，
        弯道限速更低时才减速。
        """
        p = self.p
        v = target_speed_along(points, p.a_lat_max, p.valid_max)
        return float(max(min(v, p.cruise_speed), p.v_min))

    def tick(self, target_speed, dt=0.05):
        state, event = csl_decide(self._state, target_speed, self.p)
        self._state = state
        self._commanded = ramp_speed(self._commanded, target_speed, dt, self.p)
        return {
            "state": state,
            "target_speed": float(target_speed),
            "commanded_speed": float(self._commanded),
            "event": event,
        }

    def reset(self):
        """复位到 cruise 巡航状态。"""
        self._state = CRUISE
        self._commanded = self.p.cruise_speed
