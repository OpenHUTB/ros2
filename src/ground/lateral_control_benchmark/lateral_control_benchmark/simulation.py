#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环仿真、纵向速度策略与横向性能指标。

设计要点（保证"对比公平"）：
1. **同一个纵向速度策略**：所有控制器共用 `SpeedProfile`（曲率限速 + 一阶速度响应），
   因此差异只来自横向算法；
2. **同一个初始位姿**：默认从路径起点偏移 1.2 m、航向偏 0.10 rad 起步；
3. **同一套指标**：全部由 `evaluate()` 计算，定义写在文档里。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from .controllers import BaseController
from .geometry import ReferencePath
from .vehicle_model import DynamicBicycle, KinematicBicycle, VehicleParams


# ------------------------------------------------------------------ 速度策略
@dataclass
class SpeedProfile:
    """纵向速度：巡航速度 + 按曲率限速 + 一阶加减速响应。

    曲率限速来自侧向加速度约束 ``v <= sqrt(a_lat / |kappa|)``，
    这是真实车辆过弯降速的物理依据，也避免横向控制器在急弯被"逼到极限"。
    """

    cruise_speed: float = 8.0     # 巡航速度 m/s
    min_speed: float = 2.5        # 弯道最低速度 m/s
    a_lat_max: float = 3.0        # 允许的侧向加速度 m/s^2
    accel_limit: float = 1.5      # 加速度上限 m/s^2
    decel_limit: float = 2.5      # 减速度上限 m/s^2

    def __post_init__(self) -> None:
        # 显式校验：负的巡航速度此前会被 max(..., 0) 静默钳成 0，
        # 表现为"车不动"却没有任何报错。本模块的横向指标按前进定义，
        # 不支持倒车巡航，因此在配置阶段就拒绝。
        if not math.isfinite(self.cruise_speed) or self.cruise_speed <= 0.0:
            raise ValueError(
                f"cruise_speed 必须为正数（当前 {self.cruise_speed}）；"
                "本模块不支持倒车巡航，倒车请直接用 allow_reverse 的车辆模型")
        if not math.isfinite(self.min_speed) or self.min_speed < 0.0:
            raise ValueError(f"min_speed 必须是非负数，当前 {self.min_speed}")
        if self.min_speed > self.cruise_speed:
            raise ValueError(
                f"min_speed({self.min_speed}) 不能大于 cruise_speed({self.cruise_speed})")
        if not math.isfinite(self.a_lat_max) or self.a_lat_max <= 0.0:
            raise ValueError(f"a_lat_max 必须为正数，当前 {self.a_lat_max}")
        if self.accel_limit < 0.0 or self.decel_limit < 0.0:
            raise ValueError("accel_limit 与 decel_limit 必须是非负数")

    def target(self, path: ReferencePath, s: float) -> float:
        v = self.cruise_speed
        kappa = abs(path.curvature_at_s(s))
        if kappa > 1e-6:
            v = min(v, math.sqrt(self.a_lat_max / kappa))
        return float(max(v, self.min_speed))

    def apply(self, v_now: float, v_target: float, dt: float) -> float:
        rate = self.accel_limit if v_target > v_now else self.decel_limit
        dv = float(np.clip(v_target - v_now, -rate * dt, rate * dt))
        return float(max(v_now + dv, 0.0))


def track_speed(v_target: float, v_now: float, dt: float,
                kp: float = 0.8) -> float:
    """简单的纵向 P 控制器，输出加速度指令（限幅在车辆参数内）。"""
    return float(kp * (v_target - v_now))


# ------------------------------------------------------------------ 仿真配置
@dataclass
class SimulationConfig:
    """仿真参数。"""

    dt: float = 0.02                       # 控制/积分步长，50 Hz
    duration: float = 60.0                 # 最长仿真时间 s
    initial_offset: float = 1.2            # 初始横向偏移 m（左正）
    initial_heading_error: float = 0.10    # 初始航向偏差 rad
    initial_speed: float = 3.0             # 初始车速 m/s（非零，避免静止起步瞬态主导指标）
    model: str = "kinematic"               # kinematic 或 dynamic
    finish_tolerance: float = 1.0          # 距终点该距离内视为到达
    speed: SpeedProfile = field(default_factory=SpeedProfile)
    params: VehicleParams = field(default_factory=VehicleParams)

    def __post_init__(self) -> None:
        if not math.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError(f"dt 必须为正数，当前 {self.dt}")
        if not math.isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError(f"duration 必须为正数，当前 {self.duration}")
        if not math.isfinite(self.initial_speed) or self.initial_speed < 0.0:
            raise ValueError(
                f"initial_speed 必须是非负数（当前 {self.initial_speed}）；"
                "负初速度请显式设置 params.allow_reverse=True")
        if self.model not in ("kinematic", "dynamic"):
            raise ValueError(f"model 只能是 'kinematic' 或 'dynamic'，当前 {self.model!r}")


@dataclass
class Trace:
    """一次仿真的完整时序记录。"""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    v: np.ndarray
    delta: np.ndarray
    e: np.ndarray            # 横向偏差（相对路径，左正）
    psi: np.ndarray          # 航向偏差（车头 - 路径）
    controller: str
    scenario: str
    # 被"不允许倒车"规则拦截的步数（0 表示整个仿真中速度从未试图变负）
    reverse_blocked_steps: int = 0

    def __len__(self) -> int:
        return int(self.t.shape[0])


class Simulator:
    """把车辆模型、纵向速度策略与横向控制器接成闭环。"""

    def __init__(self, path: ReferencePath, config: Optional[SimulationConfig] = None) -> None:
        self.path = path
        self.cfg = config or SimulationConfig()
        self.model = (DynamicBicycle(self.cfg.params)
                      if self.cfg.model == "dynamic"
                      else KinematicBicycle(self.cfg.params))

    def initial_state(self) -> np.ndarray:
        start = self.path.points[0]
        tangent = self.path.tangents[0]
        normal = self.path.normals[0]
        yaw = math.atan2(tangent[1], tangent[0]) + self.cfg.initial_heading_error
        pos = start + self.cfg.initial_offset * normal + 2.0 * tangent
        return np.array([pos[0], pos[1], yaw, self.cfg.initial_speed])

    def run(self, controller: BaseController, scenario: str = "") -> Trace:
        controller.reset()
        state = self.initial_state()
        memory = {"vy": 0.0, "r": 0.0}
        # 计数器按"每次 run 独立"重置，重复调用同一 Simulator 时不会累加旧值
        if hasattr(self.model, "reverse_blocked_steps"):
            self.model.reverse_blocked_steps = 0
        record = {k: [] for k in ("t", "x", "y", "yaw", "v", "delta", "e", "psi")}
        t = 0.0
        while t <= self.cfg.duration:
            projection = self.path.project(state[:2])
            v_target = self.cfg.speed.target(self.path, projection.s)
            accel = track_speed(v_target, state[3], self.cfg.dt)

            result = controller.compute(state, state[3], self.path, projection, self.cfg.dt)
            delta = result.delta

            record["t"].append(t)
            record["x"].append(float(state[0]))
            record["y"].append(float(state[1]))
            record["yaw"].append(float(state[2]))
            record["v"].append(float(state[3]))
            record["delta"].append(float(delta))
            record["e"].append(float(projection.signed_distance))
            path_yaw = math.atan2(projection.tangent[1], projection.tangent[0])
            record["psi"].append(float((state[2] - path_yaw + math.pi) % (2 * math.pi) - math.pi))

            if self.cfg.model == "dynamic":
                state, memory = self.model.step(state, delta, accel, self.cfg.dt, memory)
            else:
                state = self.model.step(state, delta, accel, self.cfg.dt)
            t += self.cfg.dt

            if not self.path.closed:
                end = self.path.points[-1]
                if float(np.hypot(state[0] - end[0], state[1] - end[1])) <= self.cfg.finish_tolerance:
                    break

        return Trace(
            t=np.array(record["t"]), x=np.array(record["x"]), y=np.array(record["y"]),
            yaw=np.array(record["yaw"]), v=np.array(record["v"]),
            delta=np.array(record["delta"]), e=np.array(record["e"]),
            psi=np.array(record["psi"]),
            controller=getattr(controller, "name", "controller"), scenario=scenario,
            reverse_blocked_steps=int(getattr(self.model, "reverse_blocked_steps", 0)),
        )


# ------------------------------------------------------------------ 指标
@dataclass
class Metrics:
    """横向跟踪性能指标（定义见文档第 5 节）。"""

    controller: str
    scenario: str
    steps: int
    duration: float
    e_mean_abs: float        # 平均绝对横向偏差
    e_rms: float             # 横向偏差 RMS
    e_max_abs: float         # 最大绝对横向偏差
    e_steady_abs: float      # 末段稳态平均绝对偏差
    psi_mean_abs_deg: float  # 平均绝对航向偏差
    delta_rate_rms_deg: float  # 转角变化率 RMS（平顺性）
    delta_max_abs_deg: float   # 最大转角
    convergence_time: float     # 首次进入并保持 0.2 m 的时刻
    off_track_ratio: float      # |e| > 1.0 m 的时间占比（安全性）
    v_mean: float
    # 被"不允许倒车"规则拦截的步数；>0 说明该场景确实触碰到了边界
    reverse_blocked_steps: int = 0

    def as_dict(self) -> Dict[str, float | str]:
        return {
            "controller": self.controller,
            "scenario": self.scenario,
            "steps": self.steps,
            "duration_s": round(self.duration, 3),
            "e_mean_abs_m": round(self.e_mean_abs, 4),
            "e_rms_m": round(self.e_rms, 4),
            "e_max_abs_m": round(self.e_max_abs, 4),
            "e_steady_abs_m": round(self.e_steady_abs, 4),
            "psi_mean_abs_deg": round(self.psi_mean_abs_deg, 3),
            "delta_rate_rms_deg_s": round(self.delta_rate_rms_deg, 2),
            "delta_max_abs_deg": round(self.delta_max_abs_deg, 3),
            "convergence_time_s": round(self.convergence_time, 3),
            "off_track_ratio": round(self.off_track_ratio, 4),
            "v_mean_mps": round(self.v_mean, 3),
            "reverse_blocked_steps": int(self.reverse_blocked_steps),
        }


def evaluate(trace: Trace, steady_ratio: float = 0.25,
             converge_threshold: float = 0.2, off_track_threshold: float = 1.0) -> Metrics:
    """从轨迹计算指标。

    * ``e_steady_abs``：轨迹末段（默认后 25%）的平均绝对横向偏差，反映稳态精度；
    * ``convergence_time``：横向偏差**首次进入并此后一直保持**在阈值内的时刻；
      若从未满足，取仿真总时长（便于区分"收敛慢"与"不收敛"）；
    * ``off_track_ratio``：|e| 超过阈值的采样占比，反映安全性；
    * ``delta_rate_rms_deg``：转角差分得到的变化率 RMS，反映转向平顺性。
    """
    e_abs = np.abs(trace.e)
    n = e_abs.shape[0]
    dt = float(trace.t[1] - trace.t[0]) if n > 1 else 0.0
    tail = max(1, int(n * steady_ratio))
    rate = np.gradient(np.degrees(trace.delta), dt) if n > 2 and dt > 0 else np.zeros(n)

    convergence = float(trace.t[-1]) if n else 0.0
    for i in range(n):
        if bool(np.all(e_abs[i:] <= converge_threshold)):
            convergence = float(trace.t[i])
            break

    return Metrics(
        controller=trace.controller,
        scenario=trace.scenario,
        steps=n,
        duration=float(trace.t[-1]) if n else 0.0,
        e_mean_abs=float(e_abs.mean()) if n else 0.0,
        e_rms=float(np.sqrt((trace.e ** 2).mean())) if n else 0.0,
        e_max_abs=float(e_abs.max()) if n else 0.0,
        e_steady_abs=float(e_abs[-tail:].mean()) if n else 0.0,
        psi_mean_abs_deg=float(np.degrees(np.abs(trace.psi)).mean()) if n else 0.0,
        delta_rate_rms_deg=float(np.sqrt((rate ** 2).mean())) if n else 0.0,
        delta_max_abs_deg=float(np.degrees(np.abs(trace.delta)).max()) if n else 0.0,
        convergence_time=convergence,
        off_track_ratio=float((e_abs > off_track_threshold).mean()) if n else 0.0,
        v_mean=float(trace.v.mean()) if n else 0.0,
        reverse_blocked_steps=int(getattr(trace, "reverse_blocked_steps", 0)),
    )


def rank_controllers(metrics: Sequence[Metrics]) -> Dict[str, Dict[str, float]]:
    """按场景对各指标排名，汇总平均名次（越小越好）。

    跨场景直接比较绝对数值会被"场景难度"带偏，因此用**名次和**做综合排序，
    这也是同类 benchmark 的常见做法。
    """
    by_scenario: Dict[str, List[Metrics]] = {}
    for m in metrics:
        by_scenario.setdefault(m.scenario, []).append(m)

    # 参与排名的指标：越小越好
    keys = ("e_mean_abs", "e_rms", "e_max_abs", "e_steady_abs",
            "psi_mean_abs_deg", "delta_rate_rms_deg", "convergence_time", "off_track_ratio")
    accum: Dict[str, Dict[str, float]] = {}
    for scenario, items in by_scenario.items():
        for key in keys:
            order = sorted(items, key=lambda m: getattr(m, key))
            for rank, m in enumerate(order, start=1):
                entry = accum.setdefault(m.controller, {})
                entry[f"{scenario}:{key}"] = float(rank)
                entry["rank_sum"] = entry.get("rank_sum", 0.0) + rank
                entry["rank_count"] = entry.get("rank_count", 0.0) + 1.0
    for name, entry in accum.items():
        entry["rank_mean"] = entry["rank_sum"] / max(entry["rank_count"], 1.0)
    return accum
