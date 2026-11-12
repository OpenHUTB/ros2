# -*- coding: utf-8 -*-
"""载重偏置下的制动：轴荷转移、抱死顺序与跑偏的准静态计算。

为什么做这个模块
----------------
仓库里已有 AEB（何时该刹车）、车辆动力性测试（加速能力）、横向控制对比（打方向），
但没有回答一个很实际的问题：**货装偏了，刹车距离会差多少、车会不会跑偏**。

本模块用纯 numpy 的准静态模型给出这个影响的三条量化结论：

1. 前后载重偏置 → 轴荷重新分配 → 固定制动力分配（bias）下先后抱死 → 制动距离变化；
2. 与"理想 EBD"（按动态轴荷分配）的差距，说明 ABS/EBD 到底买到了什么；
3. 左右载重偏置 → 左右制动力差 → 横摆力矩 → 制动跑偏量。

模型边界（写清楚，避免被当成整车仿真）
--------------------------------------
* 纵向：单自由度质点 + 动态轴荷转移，轮胎用 μ 上限（滑移率不单独建模）；
* 横向：准静态估算（左右力差 → 横摆力矩 → 一阶积分得到跑偏量），不含悬架/侧倾动力学；
* 不含 ABS 的循环控制、不含轮胎非线性（μ-滑移曲线）、不含空气升力。

单位：m、kg、N、m/s、rad、s。
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np

G = 9.81


@dataclass
class VehicleSpec:
    """整车与工况参数。默认值取一台 1.5 t 的前置前驱轿车量级。"""

    mass: float = 1500.0          # 整备质量 kg
    wheelbase: float = 2.70       # 轴距 L m
    com_from_front: float = 1.20  # 质心到前轴距离 a m
    com_height: float = 0.55      # 质心高度 h m
    track: float = 1.55           # 轮距 T m
    mu: float = 0.90              # 路面附着系数
    brake_bias: float = 0.62      # 前轴制动力占比 β
    cda: float = 0.65             # Cd·A m²
    rho: float = 1.225            # 空气密度

    # 载重（可偏置）
    payload: float = 0.0          # 载重质量 kg
    payload_x: float = 1.35       # 载重质心到前轴的距离 m（0=前轴，L=后轴）
    payload_y: float = 0.0        # 载重横向偏移 m（正=右）

    v0: float = 27.78             # 制动初速 m/s（100 km/h）

    def total_mass(self) -> float:
        return self.mass + self.payload

    def com_position(self) -> float:
        """合成质心到前轴的距离（载重偏置的直接后果）。"""
        m = self.total_mass()
        return (self.mass * self.com_from_front + self.payload * self.payload_x) / m

    def rear_distance(self) -> float:
        return self.wheelbase - self.com_position()


def static_axle_loads(spec: VehicleSpec) -> Tuple[float, float]:
    """静态轴荷 (前, 后)，N。载重后移会让前轴卸载、后轴加载。"""
    m = spec.total_mass()
    a = spec.com_position()
    b = spec.rear_distance()
    return m * G * b / spec.wheelbase, m * G * a / spec.wheelbase


def dynamic_axle_loads(spec: VehicleSpec, decel: float) -> Tuple[float, float]:
    """制动时的动态轴荷 (前, 后)，N。

    减速度 decel 为正表示正在减速；前轴加载 `m·a·h/L`，后轴等量卸载，
    后轴不允许为负（抬轮），此处截断为 0 并由此产生"后轴先抱死"。

    推导：对后轮接地点取矩，ΣM = 0 →
        Fz_f·L = m·g·b + m·a·h
    """
    m = spec.total_mass()
    a = spec.com_position()
    b = spec.rear_distance()
    transfer = m * max(decel, 0.0) * spec.com_height / spec.wheelbase
    fz_f = m * G * b / spec.wheelbase + transfer
    fz_r = m * G * a / spec.wheelbase - transfer
    return fz_f, max(fz_r, 0.0)


def axle_brake_force(spec: VehicleSpec, fz_f: float, fz_r: float,
                     demand: float) -> Tuple[float, float, bool, bool]:
    """按固定 bias 分配制动力，并按 μ·Fz 截断。

    返回 (前轴力, 后轴力, 前轴抱死?, 后轴抱死?)。demand 为纵向总需求力 N。
    """
    req_f = spec.brake_bias * demand
    req_r = (1.0 - spec.brake_bias) * demand
    cap_f = spec.mu * fz_f
    cap_r = spec.mu * fz_r
    fx_f = min(req_f, cap_f)
    fx_r = min(req_r, cap_r)
    locked_f = req_f > cap_f + 1e-9
    locked_r = req_r > cap_r + 1e-9
    return fx_f, fx_r, locked_f, locked_r


def ideal_deceleration(spec: VehicleSpec) -> float:
    """理想 EBD：制动力始终按动态轴荷分配，四轮同时到 μ 上限。"""
    return spec.mu * G


def ideal_stopping_distance(spec: VehicleSpec) -> float:
    return spec.v0 ** 2 / (2.0 * ideal_deceleration(spec))


def _brake_equilibrium(spec: VehicleSpec, demand: float, decel_guess: float,
                       drag: float, iters: int = 12, tol: float = 1e-10):
    """给定制动力需求，求"载荷—制动力—减速度"的不动点。

    review 指出的问题：如果直接用上一步（或 0）的减速度算载荷，第一步没有载荷前移，
    前轴会被瞬时压爆、错误地记为"先抱死"。所以这里对每个时间步做自洽迭代：

        载荷 ← 当前减速度  →  制动力（按 μFz 截断）  →  新减速度  →  回到载荷

    收敛后再判定抱死状态，才是稳态意义上的"谁先饱和"。
    """
    m = spec.total_mass()
    a = decel_guess
    fz_f = fz_r = 0.0
    fx_f = fx_r = 0.0
    locked_f = locked_r = False
    for _ in range(iters):
        fz_f, fz_r = dynamic_axle_loads(spec, a)
        fx_f, fx_r, locked_f, locked_r = axle_brake_force(spec, fz_f, fz_r, demand)
        a_new = (fx_f + fx_r + drag) / m
        if abs(a_new - a) < tol:
            a = a_new
            break
        a = a_new
    return a, fz_f, fz_r, fx_f, fx_r, locked_f, locked_r


def simulate_braking(spec: VehicleSpec, demand_factor: float = 1.0,
                     dt: float = 0.001, max_time: float = 20.0) -> Dict[str, float]:
    """纵向制动积分（每步做自洽载荷迭代）。

    demand 取 `demand_factor · μ · m · g`（1.0 即"踩死刹车"），
    每一步先迭代到"载荷—制动力—减速度"自洽，再按 μ 上限截断——
    这样"某个轴先饱和、减速度掉下来"才会正确地体现在抱死顺序上。
    """
    m = spec.total_mass()
    demand = demand_factor * spec.mu * m * G
    v = spec.v0
    x = 0.0
    t = 0.0
    decel = spec.mu * G          # 初值用 μg，避免第一步没有载荷转移
    peak_decel = 0.0
    first_lock = "none"
    rear_lift = False
    steps = 0

    while v > 1e-3 and t < max_time:
        drag = 0.5 * spec.rho * spec.cda * v * v
        decel, fz_f, fz_r, fx_f, fx_r, locked_f, locked_r = _brake_equilibrium(
            spec, demand, decel, drag)
        peak_decel = max(peak_decel, decel)
        if fz_r <= 1e-9:
            rear_lift = True

        if first_lock == "none":
            if locked_r and not locked_f:
                first_lock = "rear"
            elif locked_f and not locked_r:
                first_lock = "front"
            elif locked_f and locked_r:
                first_lock = "both"

        v = max(v - decel * dt, 0.0)
        x += v * dt
        t += dt
        steps += 1

    return {
        "distance_m": x,
        "time_s": t,
        "peak_decel": peak_decel,
        "mean_decel": spec.v0 ** 2 / (2.0 * x) if x > 1e-9 else float("inf"),
        "first_lock": first_lock,
        "rear_lift": rear_lift,
        "steps": steps,
    }


def simulate_braking_ideal(spec: VehicleSpec, dt: float = 0.001,
                           max_time: float = 20.0) -> Dict[str, float]:
    """理想 EBD：每一步按**当前动态轴荷**分配制动力，四轮同时到 μ 上限。

    这是本模型里可达的最短距离（比闭式 v²/(2μg) 略短，因为还叠了空气阻力），
    用作对比基准；任何固定 bias 的工况都应落在它之上。
    """
    m = spec.total_mass()
    demand = spec.mu * m * G
    v, x, t, decel = spec.v0, 0.0, 0.0, 0.0
    while v > 1e-3 and t < max_time:
        fz_f, fz_r = dynamic_axle_loads(spec, decel)
        total_fz = fz_f + fz_r
        share_f = fz_f / total_fz if total_fz > 1e-9 else 0.5
        fx_f = min(demand * share_f, spec.mu * fz_f)
        fx_r = min(demand * (1.0 - share_f), spec.mu * fz_r)
        drag = 0.5 * spec.rho * spec.cda * v * v
        decel = (fx_f + fx_r + drag) / m
        v = max(v - decel * dt, 0.0)
        x += v * dt
        t += dt
    return {"distance_m": x, "time_s": t, "peak_decel": decel}


def simulate_lateral(spec: VehicleSpec, cornering_front: float = 80000.0,
                     cornering_rear: float = 90000.0, dt: float = 0.001,
                     stop_speed: float = 1.0) -> Dict[str, float]:
    """左右载重偏置导致的制动跑偏：3 自由度（vx, vy, r）积分。

    侧向只用线性轮胎（Fy = −Cα·α），纵向仍用 μ·Fz 截断——因为这里关心的是
    "左右制动力不等会把车带偏多少"，而不是极限失稳。制动到 `stop_speed` 结束，
    返回横摆角、横向位移与横摆角速度。

    若左右完全对称，ΔFx = 0，模型自然给出零跑偏（这条写进了单元测试）。
    """
    m = spec.total_mass()
    a = spec.com_position()
    b = spec.rear_distance()
    iz = m * (spec.track ** 2 + spec.wheelbase ** 2) / 12.0
    demand = spec.mu * m * G

    # 左右静态载荷分配（载重横向偏移 dy 使一侧加载）
    share_right = min(max(0.5 + spec.payload_y / spec.track, 0.05), 0.95)
    fz_total = m * G

    vx, vy, r, psi, y = spec.v0, 0.0, 0.0, 0.0, 0.0
    t = 0.0
    while vx > stop_speed and t < 30.0:
        fz_f, fz_r = dynamic_axle_loads(spec, abs(demand / m))
        # 每侧每轴的载荷：静态横向分配 × 该轴占比
        fz_f_r = fz_f * share_right
        fz_f_l = fz_f * (1.0 - share_right)
        fz_r_r = fz_r * share_right
        fz_r_l = fz_r * (1.0 - share_right)

        # 每侧制动力：先按 bias 分到轴，再按 1/2 分到左右，最后按 μ·Fz 截断
        fx_f_r = min(spec.brake_bias * demand * 0.5, spec.mu * fz_f_r)
        fx_f_l = min(spec.brake_bias * demand * 0.5, spec.mu * fz_f_l)
        fx_r_r = min((1 - spec.brake_bias) * demand * 0.5, spec.mu * fz_r_r)
        fx_r_l = min((1 - spec.brake_bias) * demand * 0.5, spec.mu * fz_r_l)

        fx_total = fx_f_r + fx_f_l + fx_r_r + fx_r_l
        mz = (fx_f_r - fx_f_l) * spec.track / 2 + (fx_r_r - fx_r_l) * spec.track / 2

        # 线性轮胎
        v_safe = max(vx, 1.0)
        alpha_f = -(vy + a * r) / v_safe
        alpha_r = -(vy - b * r) / v_safe
        fy_f = cornering_front * alpha_f
        fy_r = cornering_rear * alpha_r
        cap_f = spec.mu * fz_f * 0.95
        cap_r = spec.mu * fz_r * 0.95
        fy_f = max(-cap_f, min(cap_f, fy_f))
        fy_r = max(-cap_r, min(cap_r, fy_r))

        ax = -(fx_total + 0.5 * spec.rho * spec.cda * vx * vx) / m
        ay = (fy_f + fy_r) / m - r * vx
        r_dot = (a * fy_f - b * fy_r + mz) / iz

        vx += ax * dt
        vy += ay * dt
        r += r_dot * dt
        psi += r * dt
        y += (vy + vx * psi) * dt
        t += dt

    return {
        "yaw_deg": math.degrees(psi),
        "yaw_rate_deg_s": math.degrees(r),
        "drift_m": y,
        "time_s": t,
        "brake_force_diff": (fx_f_r + fx_r_r) - (fx_f_l + fx_r_l),
    }


def compare_with_ideal(spec: VehicleSpec) -> Dict[str, float]:
    """把当前工况与"理想 EBD"对比，给出百分比差距。"""
    real = simulate_braking(spec)
    ideal = simulate_braking_ideal(spec)["distance_m"]   # 与 cli / 图表口径一致（含风阻）
    return {
        "distance_m": real["distance_m"],
        "ideal_m": ideal,
        "gap_m": real["distance_m"] - ideal,
        "gap_pct": (real["distance_m"] - ideal) / ideal * 100.0,
        "first_lock": real["first_lock"],
    }


def sweep_payload_position(spec: VehicleSpec, positions=(0.3, 0.8, 1.35, 1.9, 2.4),
                           payload: float = 300.0) -> list:
    """扫描载重前后位置，返回每档的结果（用于画图与表格）。"""
    out = []
    for x in positions:
        s = VehicleSpec(**{**spec.__dict__, "payload": payload, "payload_x": x})
        r = simulate_braking(s)
        fz_f, fz_r = static_axle_loads(s)
        out.append({
            "payload_x": x,
            "com_x": s.com_position(),
            "fz_front": fz_f,
            "fz_rear": fz_r,
            "distance_m": r["distance_m"],
            "first_lock": r["first_lock"],
            "rear_lift": r["rear_lift"],
        })
    return out


def sweep_bias(spec: VehicleSpec, biases=(0.50, 0.55, 0.60, 0.62, 0.65, 0.70),
               payload: float = 300.0, payload_x: float = 2.4) -> list:
    """扫描制动力分配系数，看在"载重后移"工况下 bias 该怎么调。"""
    out = []
    for beta in biases:
        s = VehicleSpec(**{**spec.__dict__, "payload": payload,
                           "payload_x": payload_x, "brake_bias": beta})
        r = simulate_braking(s)
        out.append({"brake_bias": beta, "distance_m": r["distance_m"],
                    "first_lock": r["first_lock"]})
    return out
