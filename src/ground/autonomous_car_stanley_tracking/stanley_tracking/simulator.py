#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""差速驱动底盘的闭环仿真与跟踪性能指标。

仿真器按固定步长积分运动学模型：

    x'     = v * cos(theta)
    y'     = v * sin(theta)
    theta' = omega = v * tan(delta) / L

并支持两类干扰，用来检验控制器的鲁棒性：

* 初始位姿偏差（横向、航向）；
* 常值侧向风扰（等效于对横向速度叠加偏置）。

指标统计只在车辆进入参考路径附近之后开始，避免把起步段的
正常收敛过程计入稳态误差。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .controllers import BaseController, ControlCommand
from .geometry import ReferencePath, wrap_to_pi


@dataclass
class SpeedProfile:
    """纵向速度曲线：默认巡航速度 + 按曲率自动限速。

    曲率限速取自侧向加速度约束 ``v <= sqrt(a_lat_max / |kappa|)``，
    这与真实车辆"弯道减速"的横向控制前提一致。
    """

    target_speed: float = 5.0
    min_speed: float = 1.5
    a_lat_max: float = 2.5
    use_curvature_limit: bool = True

    def speed_for(self, path: ReferencePath, projection) -> float:
        speed = self.target_speed
        if self.use_curvature_limit:
            curvature = abs(projection.curvature)
            if curvature > 1e-6:
                speed = min(speed, math.sqrt(self.a_lat_max / curvature))
        return float(max(speed, self.min_speed))


@dataclass
class SimulationConfig:
    """仿真配置。"""

    dt: float = 0.02               # 控制与积分步长，50 Hz
    wheelbase: float = 2.7         # 轴距
    initial_state: Optional[np.ndarray] = None  # (x, y, yaw)
    lateral_wind: float = 0.0      # 常值侧向风扰，等价横向速度偏置 m/s
    speed_profile: SpeedProfile = field(default_factory=SpeedProfile)
    finish_tolerance: float = 1.0  # 距离参考路径终点多少米内视为到达
    max_time: float = 240.0


@dataclass
class SimulationTrace:
    """一次仿真的完整记录。"""

    time: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    speed: np.ndarray
    steering: np.ndarray
    yaw_rate: np.ndarray
    cross_track_error: np.ndarray
    heading_error: np.ndarray
    reference_xy: np.ndarray
    controller: str = ""
    scenario: str = ""

    def __len__(self) -> int:
        return int(self.time.shape[0])


class Simulator:
    """闭环仿真器：把控制器、纵向速度曲线和底盘运动学接在一起。"""

    def __init__(self, path: ReferencePath, config: Optional[SimulationConfig] = None) -> None:
        self.path = path
        self.config = config or SimulationConfig()

    def initial_state(self) -> np.ndarray:
        """默认初始位姿：路径起点沿切向，横向偏移 1.5 m、航向偏差 0.1 rad。"""
        if self.config.initial_state is not None:
            return np.asarray(self.config.initial_state, dtype=float).copy()
        start = self.path.points[0]
        tangent = self.path.tangents[0]
        normal = self.path.normals[0]
        yaw = math.atan2(tangent[1], tangent[0]) + 0.10
        point = start + 1.5 * normal + 2.0 * tangent
        return np.array([point[0], point[1], yaw])

    def run(self, controller: BaseController, scenario: str = "") -> SimulationTrace:
        """跑完整条参考路径，返回轨迹记录。"""
        cfg = self.config
        controller.reset()
        state = self.initial_state()
        speed = cfg.speed_profile.target_speed
        steering = 0.0

        records: List[Dict[str, float]] = []
        reference: List[np.ndarray] = []
        time = 0.0
        step = 0
        # 单调前向投影的起始线段：初始位姿最近的线段
        hint_index = self.path.project(state[:2]).index
        # 一个控制步内前进不会超过 speed*dt，按此设窗口即可，兼顾稳健与性能
        forward_window = max(20, int(cfg.speed_profile.target_speed * cfg.dt * 20) + 20)

        while time <= cfg.max_time:
            projection = self.path.project_forward(state[:2], hint_index,
                                                  backward=2, forward=forward_window)
            hint_index = projection.index
            speed_target = cfg.speed_profile.speed_for(self.path, projection)
            # 纵向一阶响应：避免速度指令阶跃，贴近真实底盘
            speed += float(np.clip(speed_target - speed, -2.0 * cfg.dt, 2.0 * cfg.dt))

            command: ControlCommand = controller.compute(state, speed, self.path, projection, cfg.dt)
            steering = command.steering
            yaw_rate = command.yaw_rate

            error = projection.signed_distance
            records.append(dict(
                t=time, x=state[0], y=state[1], yaw=float(wrap_to_pi(state[2])),
                v=speed, delta=steering, omega=yaw_rate,
                e=error, psi=command.heading_error,
            ))
            reference.append(projection.point)

            # 运动学积分（前向欧拉；步长 0.02 s 时离散误差可忽略）
            lateral_wind = cfg.lateral_wind
            state[0] += (speed * math.cos(state[2])) * cfg.dt
            state[1] += (speed * math.sin(state[2]) + lateral_wind) * cfg.dt
            state[2] += yaw_rate * cfg.dt
            time += cfg.dt
            step += 1

            if self._finished(state, step):
                break

        trace = SimulationTrace(
            time=np.array([r["t"] for r in records]),
            x=np.array([r["x"] for r in records]),
            y=np.array([r["y"] for r in records]),
            yaw=np.array([r["yaw"] for r in records]),
            speed=np.array([r["v"] for r in records]),
            steering=np.array([r["delta"] for r in records]),
            yaw_rate=np.array([r["omega"] for r in records]),
            cross_track_error=np.array([r["e"] for r in records]),
            heading_error=np.array([r["psi"] for r in records]),
            reference_xy=np.array(reference),
            controller=getattr(controller, "name", "controller"),
            scenario=scenario,
        )
        return trace

    def _finished(self, state: np.ndarray, step: int) -> bool:
        """终点判据：非闭合路径下，到达终点附近即停止。"""
        if self.path.closed:
            return False
        end = self.path.points[-1]
        if float(np.hypot(state[0] - end[0], state[1] - end[1])) <= self.config.finish_tolerance:
            return True
        return False


@dataclass
class Metrics:
    """跟踪性能指标。"""

    controller: str
    scenario: str
    steps: int
    duration: float
    cross_track_mean_abs: float
    cross_track_rms: float
    cross_track_max_abs: float
    cross_track_steady_abs: float
    heading_mean_abs_deg: float
    steering_rate_max_deg: float
    steering_rate_rms_deg: float
    convergence_time: float
    speed_mean: float

    def as_row(self) -> Dict[str, float | str]:
        return {
            "controller": self.controller,
            "scenario": self.scenario,
            "steps": self.steps,
            "duration_s": round(self.duration, 3),
            "cross_track_mean_abs_m": round(self.cross_track_mean_abs, 4),
            "cross_track_rms_m": round(self.cross_track_rms, 4),
            "cross_track_max_abs_m": round(self.cross_track_max_abs, 4),
            "cross_track_steady_abs_m": round(self.cross_track_steady_abs, 4),
            "heading_mean_abs_deg": round(self.heading_mean_abs_deg, 3),
            "steering_rate_max_deg_s": round(self.steering_rate_max_deg, 2),
            "steering_rate_rms_deg_s": round(self.steering_rate_rms_deg, 2),
            "convergence_time_s": round(self.convergence_time, 3),
            "speed_mean_mps": round(self.speed_mean, 3),
        }


def evaluate(trace: SimulationTrace, steady_ratio: float = 0.25) -> Metrics:
    """从仿真轨迹计算跟踪指标。

    ``cross_track_steady_abs`` 取后 ``steady_ratio`` 段（默认后 25%）的平均绝对偏差，
    代表稳态性能；``convergence_time`` 取横向偏差首次进入并保持 0.2 m 以内的时刻。
    """
    e = np.abs(trace.cross_track_error)
    n = e.shape[0]
    tail = max(1, int(n * steady_ratio))
    dt = float(trace.time[1] - trace.time[0]) if n > 1 else 0.0

    steering_rate = np.gradient(trace.steering, dt) if n > 2 and dt > 0 else np.zeros(n)

    threshold = 0.2
    convergence = float(trace.time[-1])
    for i in range(n):
        if np.all(e[i:] <= threshold):
            convergence = float(trace.time[i])
            break

    return Metrics(
        controller=trace.controller,
        scenario=trace.scenario,
        steps=n,
        duration=float(trace.time[-1]) if n else 0.0,
        cross_track_mean_abs=float(e.mean()) if n else 0.0,
        cross_track_rms=float(np.sqrt((trace.cross_track_error ** 2).mean())) if n else 0.0,
        cross_track_max_abs=float(e.max()) if n else 0.0,
        cross_track_steady_abs=float(e[-tail:].mean()) if n else 0.0,
        heading_mean_abs_deg=float(np.degrees(np.abs(trace.heading_error)).mean()) if n else 0.0,
        steering_rate_max_deg=float(np.degrees(np.abs(steering_rate)).max()) if n else 0.0,
        steering_rate_rms_deg=float(np.degrees(np.sqrt((steering_rate ** 2).mean()))) if n else 0.0,
        convergence_time=convergence,
        speed_mean=float(trace.speed.mean()) if n else 0.0,
    )
