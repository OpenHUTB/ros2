#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TTC 型 AEB 控制器（纯逻辑，不依赖 mujoco）。

控制律（与文档 §2.2 一致）：

    TTC = d / (v_ego - v_lead)            # 相对速度 <= 0 时无风险，TTC 视为无穷大
    brake = gain * (1 - TTC / TTC_th)     # TTC < TTC_th 才介入，限幅到 [0, 1]

低速保护：|v_ego| < v_stop 时不输出制动，避免停车后原地抖动。

**为什么还要一个"制动保持"（latch）**：只按比例律算，制动会抬高 TTC，TTC 抬高又
把制动力压回去，形成负反馈——实测 5 m/s / 32.6 m 间距工况下制动力长期只有 0.15，
车以约 0.9 m/s² 缓慢逼近，到 12 s 仍未停住，本质上退化成"贴脸才刹"。
AEB 的工程做法是**触发后保持一个制动下限**（真实车辆也常有 prefill/latch 逻辑）：
一旦触发，制动力不低于 `latch_floor`，直到车速降到低速保护阈值以下，
或 TTC 恢复到明显安全（> `release_ratio × 阈值`）才解除。这样减速单调、能留出安全间距。

保持功能可以关掉（`latch=False`）以复现纯比例律，便于对照。
"""
from __future__ import annotations

INF_TTC = 99.0


class AEBController:
    """比例制动控制器：TTC 越接近 0，制动力越大（可选制动保持）。"""

    #: 无碰撞风险时使用的 TTC 代表值（同时作为类属性暴露，方便调用方引用）
    INF_TTC = INF_TTC

    def __init__(self, ttc_threshold=5.0, gain=1.4, v_stop=0.1,
                 latch=True, latch_floor=0.35, release_ratio=1.5):
        self.ttc_threshold = float(ttc_threshold)
        self.gain = float(gain)
        self.v_stop = float(v_stop)
        self.latch = bool(latch)
        self.latch_floor = float(latch_floor)
        self.release_ratio = float(release_ratio)
        self.reset()

    # ---------------------------------------------------------------- 状态
    def reset(self):
        """清空触发状态（本控制器无积分项）。"""
        self.last_brake = 0.0
        self.triggered = False

    # ---------------------------------------------------------------- 核心
    def get_brake(self, ttc: float, v_ego: float, closing_speed=None) -> float:
        """由 TTC 与自车速度算制动量，返回 [0, 1]。

        ``closing_speed``（可选）= 自车速度 − 前车速度。给出时，**只有在已经不再
        接近前车时才允许解除制动保持**：否则低速段 TTC 会被"间距/极小速度"放大，
        刚触发就又解除，车会以零点几 m/s 一直往前蹭（实测 baseline 卡在 0.42 m/s
        滑行 12 s 都没停下）。
        """
        v_ego = float(v_ego)
        ttc = float(ttc)

        # 低速保护：已经停住（或几乎停住）就不再施加制动
        if abs(v_ego) < self.v_stop:
            self.last_brake = 0.0
            return 0.0

        # 解除保持的条件：TTC 明显恢复 + （若有相对速度信息）已不再接近
        if self.triggered:
            safe = ttc > self.ttc_threshold * self.release_ratio
            not_closing = (closing_speed is None) or (float(closing_speed) <= 0.0)
            if safe and not_closing:
                self.triggered = False

        brake = 0.0
        if 0.0 < ttc < self.ttc_threshold:
            raw = self.gain * (1.0 - ttc / self.ttc_threshold)
            # 只有比例项真的 > 0 才算"触发"：gain=0 等价于关掉 AEB，
            # 此时若仍置 triggered，制动保持会把下限补上，对照实验就失效了
            # （实测踩过：baseLine 与 no_aeb 指标完全相同，等于没有对照组）。
            if raw > 0.0:
                brake = max(0.0, min(1.0, raw))
                self.triggered = True

        # 触发过就保持一个下限（制动保持）
        if self.latch and self.triggered:
            brake = max(brake, self.latch_floor)

        self.last_brake = brake
        return brake

    # ---------------------------------------------------------------- 工具
    @staticmethod
    def compute_ttc(gap: float, v_ego: float, v_lead: float = 0.0,
                    v_stop: float = 0.1) -> float:
        """TTC = 间距 / 相对速度；相对速度 <= 0（不再接近）时返回 ``INF_TTC``。

        "没有碰撞风险"必须用一个**明显不可达**的值表示，而不是 0 或负数，
        否则下游很容易把"安全"当成"马上要撞"。
        """
        rel = float(v_ego) - float(v_lead)
        if abs(float(v_ego)) < v_stop:
            return INF_TTC
        if rel <= 1e-6:
            return INF_TTC
        return float(gap) / rel
