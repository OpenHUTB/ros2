#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offroad_core.py —— 车辆动力学 · 越野地形共享核心
====================================================
同一套『轮胎滑移 + ABS 制动 + 越野地形』模型被两个平台复用：
  - mujoco_offroad_dynamics.py : MuJoCo 渲染版（本机装 mujoco 后运行）
  - sim_2d_offroad_dynamics.py : 2D 轻量 matplotlib 仿真版（可直接跑，出效果动图）

物理要点（纵向动力学 + 轮胎滑移）：
  - 单轮等效模型：车速 v、轮面速度 v_w = ω·r、滑移率 λ = (v - v_w)/v（制动时 0~1，
    λ=0 纯滚动、λ=1 轮锁死）。
  - 轮胎-地面摩擦系数 μ(λ) 随滑移率先升后降：在 λ_opt≈0.2 处达峰值 μ_peak，
    之后衰减到滑动摩擦 μ_slip（λ=1）。峰值 μ 对应最大制动力，也是转向可控的区间。
  - 越野地形：地面摩擦 μ_peak 随位置变化（干路/泥地/雪地），并可叠加坡度 θ。
  - 无 ABS：直接给大制动压力 → 轮锁死 λ→1 → μ 掉到滑动摩擦，制动力弱、制动距离长；
    ABS：实时把 λ 调回 λ_opt（峰值 μ）→ 制动力最大、制动距离最短、保留转向能力。

只依赖 numpy，可在任意 Python 环境复用。
"""

import numpy as np


# ================================================================ 轮胎滑移模型
def tire_mu(lam, mu_peak, mu_slip, lam_opt=0.2):
    """滑移率-摩擦系数曲线 μ(λ)。
    lam<=lam_opt 线性上升到 mu_peak；lam>lam_opt 线性衰减到 mu_slip(λ=1)。"""
    lam = float(np.clip(lam, 0.0, 1.0))
    if lam <= lam_opt:
        return mu_slip + (mu_peak - mu_slip) * (lam / lam_opt)
    return mu_peak - (mu_peak - mu_slip) * (lam - lam_opt) / (1.0 - lam_opt)


# ================================================================ 越野地形
class OffroadTerrain:
    """越野地形：按路程分段的摩擦剖面 + 坡度。返回 (mu_peak, theta_deg)。"""

    def __init__(self):
        # 每段: (起点路程, 终点路程, 峰值摩擦 μ_peak, 坡度 deg)
        self.segments = [
            (0.0, 40.0, 0.90, 0.0),    # 干沥青路
            (40.0, 90.0, 0.50, 0.0),   # 泥地
            (90.0, 140.0, 0.30, 0.0),  # 积雪/冰面（低摩擦，ABS 收益最明显）
            (140.0, 170.0, 0.60, 6.0), # 上坡越野段
        ]
        self.total = 170.0

    def at(self, s):
        for (a, b, mu, th) in self.segments:
            if a <= s < b:
                return mu, th
        return 0.60, 0.0

    def label_at(self, s):
        if s < 40.0:
            return "干路"
        if s < 90.0:
            return "泥地"
        if s < 140.0:
            return "雪地"
        return "上坡"


# ================================================================ 车辆动力学
class VehicleDynamics:
    """单轮等效纵向制动动力学。
    state: v 车速(m/s), w 轮角速度(rad/s), s 路程(m)。"""

    def __init__(self, m=1500.0, r=0.3, inertia=1.5, t_max=1.2e4):
        self.m = float(m)          # 整车等效质量 kg
        self.r = float(r)          # 车轮半径 m
        self.I = float(inertia)    # 车轮等效转动惯量
        self.t_max = float(t_max)  # 最大制动扭矩 N·m（足以锁死车轮）
        self.g = 9.81

    def reset(self, v0):
        self.v = float(v0)
        self.w = float(v0 / self.r)      # 初始纯滚动
        self.s = 0.0

    def step(self, pressure, dt, mu_peak, theta_deg=0.0):
        """推进一帧。pressure∈[0,1] 制动压力（ABS 在其内部调压）。
        返回 (λ, μ_used)。"""
        theta = np.radians(theta_deg)
        N = self.m * self.g * np.cos(theta)          # 法向力
        slope = self.m * self.g * np.sin(theta)      # 坡道力（上坡为阻力）

        lam = max((self.v - self.w * self.r) / max(self.v, 1e-3), 0.0)
        lam = float(np.clip(lam, 0.0, 1.0))
        mu = tire_mu(lam, mu_peak, 0.6 * mu_peak)    # 当前可用摩擦（滑动摩擦≈0.6峰值，低摩擦路面更甚）
        F_x = mu * N                                  # 地面制动力（反向）

        # 车轮角加速度：制动扭矩 vs 地面驱动扭矩
        w_dot = (-pressure * self.t_max + F_x * self.r) / self.I
        # 车辆加速度：地面制动力 + 坡道阻力
        v_dot = (-F_x - slope) / self.m
        self.w += w_dot * dt
        self.v += v_dot * dt
        self.s += self.v * dt
        self.v = max(self.v, 0.0)
        return lam, mu


# ================================================================ ABS 控制器
class ABS:
    """防抱死制动：把滑移率 λ 调到峰值摩擦对应的 λ_opt，最大化制动力、避免锁死。
    简单的阈值式调压：λ 超调则减压、回落后增压（bang-bang）。"""

    def __init__(self, lam_opt=0.2, p_min=0.02, p_max=1.0):
        self.lam_opt = lam_opt
        self.p_min = p_min
        self.p_max = p_max
        self.pressure = 0.0

    def control(self, lam):
        if lam > self.lam_opt + 0.05:
            self.pressure = max(self.p_min, self.pressure - 0.08)
        elif lam < self.lam_opt - 0.05:
            self.pressure = min(self.p_max, self.pressure + 0.06)
        return self.pressure


# ================================================================ 制动对比
def run_brake(vehicle, terrain, use_abs, v0=20.0, dt=0.002, t_max=12.0):
    """在越野地形上以 v0 紧急制动，返回 (停止路程, 停止时间, 轨迹记录)。
    use_abs=True 用 ABS 滑移率控制，False 为固定大压力（无 ABS，轮锁死打滑）。"""
    veh = VehicleDynamics(**vehicle) if isinstance(vehicle, dict) else vehicle
    veh.reset(v0)
    abs_ctrl = ABS() if use_abs else None
    pressure = 0.8 if not use_abs else 0.0
    rec = []
    t = 0.0
    while veh.v > 0.05 and t < t_max:
        mu_p, theta = terrain.at(veh.s)
        if use_abs:
            # 预测：先推进半步估 λ，再调压（简单两步）
            lam, mu = veh.step(pressure, dt, mu_p, theta)
            pressure = abs_ctrl.control(lam)
        else:
            lam, mu = veh.step(pressure, dt, mu_p, theta)
        rec.append((t, veh.s, veh.v, lam, mu, mu_p))
        t += dt
    stop_s = veh.s if veh.v <= 0.05 else float("nan")
    return stop_s, t, rec
