"""列车纵向动力学：牵引、制动、基本阻力与坡道附加阻力。

只算纵向，不含垂向与横向，这也是 ATO/ATP 仿真里常用的简化程度。
参数按国内 6 辆编组动车组取值，量级对齐实际（启动加速度 0.6 m/s² 上下，
常用制动 1.0 m/s²，最高运行速度 160 km/h）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.81


@dataclass
class TrainSpec:
    """一列车的机械与阻力参数。"""

    mass: float = 300.0e3        # kg，6 辆编组满载约 300 t
    rotary_factor: float = 1.06  # 回转质量系数 γ，把轮对转动惯量折算进平动质量
    max_tractive_effort: float = 200.0e3   # N，启动牵引力上限
    rated_power: float = 3.2e6   # W，牵引功率，高速段按恒功率出力
    service_brake_decel: float = 1.0    # m/s²，常用制动（电空复合）
    emergency_brake_decel: float = 1.2  # m/s²，紧急制动（纯空气）
    brake_build_up: float = 0.8  # s，制动力上升时间常数（空走时间的近似）

    # Davis 基本阻力 R(v) = a + b·v + c·v²，单位 N
    davis_a: float = 1.5e3
    davis_b: float = 20.0
    davis_c: float = 6.0

    def traction_force(self, v: float) -> float:
        """当前速度下能给出的最大牵引力，恒力矩段与恒功率段取小。"""
        if v < 1e-6:
            return self.max_tractive_effort
        return min(self.max_tractive_effort, self.rated_power / v)

    def resistance(self, v: float) -> float:
        """基本阻力，与速度的二次多项式拟合。"""
        return self.davis_a + self.davis_b * v + self.davis_c * v * v

    def grade_force(self, grade: float) -> float:
        """坡道附加阻力，上坡为正。grade 用千分数（‰），如 6 表示 6‰。"""
        return self.mass * G * math.sin(math.atan(grade / 1000.0))

    def accel_capability(self, v: float, grade: float = 0.0) -> float:
        """当前速度下还能给出的加速度（已扣掉阻力与坡道），用于速度曲线前向积分。"""
        net = self.traction_force(v) - self.resistance(v) - self.grade_force(grade)
        return net / (self.rotary_factor * self.mass)


@dataclass
class Segment:
    """一段线路。start/end 为里程（m），limit_kph 为该段限速。"""

    start: float
    end: float
    limit_kph: float
    grade: float = 0.0   # ‰，上坡为正

    @property
    def length(self) -> float:
        return self.end - self.start


def speed_limit_at(segments: list[Segment], x: float) -> float:
    """里程 x 处的静态限速（m/s）。落在段外时按最近一段算。"""
    for seg in segments:
        if seg.start <= x < seg.end:
            return seg.limit_kph / 3.6
    if x < segments[0].start:
        return segments[0].limit_kph / 3.6
    return segments[-1].limit_kph / 3.6


def grade_at(segments: list[Segment], x: float) -> float:
    for seg in segments:
        if seg.start <= x < seg.end:
            return seg.grade
    return segments[-1].grade


def braking_distance(v0: float, decel: float) -> float:
    """从 v0 用恒定减速度刹停所需的距离。"""
    if decel <= 0.0:
        return float('inf')
    return v0 * v0 / (2.0 * decel)


def energy_kwh(spec: TrainSpec, v: float, dt: float) -> float:
    """一步牵引能耗（kWh），只算正向牵引做功，再生制动不计入。"""
    force = spec.traction_force(v)
    return force * v * dt / 3.6e6
