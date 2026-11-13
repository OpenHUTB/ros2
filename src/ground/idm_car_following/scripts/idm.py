"""智能驾驶员模型（Intelligent Driver Model, IDM）跟车仿真。

IDM 是一类经典的微观交通流"跟驰模型"，描述车辆依据与前方车辆的距离 s
和速度差 Δv 连续调整自身加速度：

    dv/dt = a * [ 1 - (v/v0)^delta - (s*/s)^2 ]

    s* = s0 + max(0, v*T + v*Δv / (2*sqrt(a*b)))     （Δv = v - v_lead）

参数:
    v0    : 期望速度 (m/s)
    T     : 安全车头时距 (s)
    a     : 最大加速度 (m/s^2)
    b     : 舒适减速度 (m/s^2)
    s0    : 静止最小间距 (m)
    delta : 加速度指数（常取 4）

无前车（s 很大）时 s*/s -> 0，车辆平滑加速到 v0；跟车稳态时
s -> s0 + v*T，形成稳定车队。
"""

import math


class IDM:
    def __init__(self, v0=15.0, T=1.5, a=2.0, b=3.0, s0=2.0, delta=4):
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.s0 = s0
        self.delta = delta

    def accel(self, v, v_lead, gap):
        """返回加速度 dv/dt。gap 为保险杠到保险杠间距 (m)。"""
        gap = max(gap, 0.1)
        dv = v - v_lead
        s_star = self.s0 + max(0.0, v * self.T + v * dv / (2.0 * math.sqrt(self.a * self.b)))
        free = 1.0 - (v / self.v0) ** self.delta
        inter = (s_star / gap) ** 2
        return self.a * (free - inter)

    def equilibrium_gap(self, v):
        """稳态跟驰（Δv=0）下，速度 v 对应的平衡间距。

        由 accel=0 得 (s*/s)^2 = 1-(v/v0)^delta，故
            s = (s0 + v*T) / sqrt(1-(v/v0)^delta)
        当 v -> v0 时 free -> 0，平衡间距 -> +inf（无法在期望速度上稳定跟驰）。
        """
        free = 1.0 - (v / self.v0) ** self.delta
        if free <= 1e-9:
            return float("inf")
        s_star = self.s0 + v * self.T
        return s_star / math.sqrt(free)


def simulate_single(idm, lead_speed_fn, T, dt=0.01, ego_v0=12.0, gap0=30.0, veh_len=5.0):
    """单跟随车跟驰一组前车速度曲线。返回 t, gap, v_ego, v_lead。"""
    n = int(T / dt)
    lead_pos = gap0 + veh_len   # 令初始 gap = gap0（保险杠间距）
    ego_pos = 0.0
    v_ego = ego_v0
    ts, gaps, ve, vl = [0.0], [], [], []
    for k in range(n):
        t = k * dt
        v_lead = lead_speed_fn(t)
        gap = lead_pos - ego_pos - veh_len
        a_ego = idm.accel(v_ego, v_lead, gap)
        # 记录
        gaps.append(gap)
        ve.append(v_ego)
        vl.append(v_lead)
        # 欧拉积分
        lead_pos += v_lead * dt
        ego_pos += v_ego * dt
        v_ego += a_ego * dt
        if v_ego < 0:
            v_ego = 0.0
        ts.append(t + dt)
    gaps.append(lead_pos - ego_pos - veh_len)
    ve.append(v_ego)
    vl.append(lead_speed_fn(T))
    return ts, gaps, ve, vl


def simulate_platoon(idm, lead_speed_fn, n_follow, T, dt=0.01, v_init=15.0,
                     gap_init=25.0, veh_len=5.0):
    """车队仿真：编号 0 为领航车（速度由 lead_speed_fn 给定），1..n 为 IDM 跟随车。

    返回 t, speeds(按车编号的列表 of 列表)。
    """
    m = n_follow + 1
    pos = [0.0] * m
    # 初始均匀排布（车头 i 在前）
    for i in range(m):
        pos[i] = -i * (gap_init + veh_len)
    v = [v_init] * m
    n = int(T / dt)
    ts = [0.0]
    speeds = [[v[i]] for i in range(m)]
    for k in range(n):
        t = k * dt
        v_lead = lead_speed_fn(t)
        v[0] = v_lead
        newv = v[:]
        for i in range(1, m):
            gap = pos[i - 1] - pos[i] - veh_len
            newv[i] = v[i] + idm.accel(v[i], v[i - 1], gap) * dt
            if newv[i] < 0:
                newv[i] = 0.0
        # 位置推进（用旧速度）
        for i in range(m):
            pos[i] += v[i] * dt
        v = newv
        for i in range(m):
            speeds[i].append(v[i])
        ts.append(t + dt)
    return ts, speeds
