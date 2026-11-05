#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mujoco_domain_rand.py —— Sim-to-Real 域随机化 · MuJoCo 渲染版
================================================================================
在 MuJoCo 差速小车环境中验证『域随机化（Domain Randomization）』的价值：
同一台小车，物理参数（车身质量 m、车轮摩擦 c、电机增益 k）被『域随机化』，
即部署时的真实参数与仿真标称不同。用『前馈控制器 u = g·v_ref』做速度跟踪：
  - 标称标定 g_nom：只用仿真标称参数标定（搬去真实参数即失效）；
  - 域随机化标定 g_dr：在真实参数分布上采样标定（对整段分布鲁棒）。
比较两者在真实参数上的速度跟踪误差。

核心标定/评估逻辑与 2D 轻量版 sim_2d_domain_rand.py 共用 domain_rand_core.py；
本文件只是把『被控对象』换成 MuJoCo 的真实动力学渲染。

使用前提（本机）：pip install mujoco numpy
运行:
  python3 mujoco_domain_rand.py
说明：本项目为课程/学习用途的仿真代码，不含真实道路行驶与真车控制，遵循开源许可。
"""

import numpy as np
import mujoco

from domain_rand_core import (SecondOrderVehicle, FeedforwardController,
                              ref_speed, DomainRandomizer,
                              calibrate_nominal, calibrate_domain_randomized,
                              evaluate)

# ================================================================ MuJoCo 小车模型
MUJOCO_XML = r"""
<mujoco model="domain_rand_car">
  <option timestep="0.005" integrator="Euler"/>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1"
          friction="0.9 0.05 0.001" condim="3"/>
    <body name="chassis" pos="0 0 0.12">
      <freejoint name="root"/>
      <geom name="body" type="box" size="0.22 0.14 0.06" mass="{MASS}" friction="{BODY_FRI}"/>
      <body name="wheel_fl" pos="0.18 0.15 0.05" euler="0 90 0">
        <joint name="wfl" type="hinge" axis="0 1 0" frictionloss="{WHEEL_FRI}"/>
        <geom name="wfl_g" type="cylinder" size="0.05 0.025" mass="0.08" friction="{WHEEL_FRI}"/>
      </body>
      <body name="wheel_fr" pos="0.18 -0.15 0.05" euler="0 90 0">
        <joint name="wfr" type="hinge" axis="0 1 0" frictionloss="{WHEEL_FRI}"/>
        <geom name="wfr_g" type="cylinder" size="0.05 0.025" mass="0.08" friction="{WHEEL_FRI}"/>
      </body>
      <body name="wheel_rl" pos="-0.18 0.15 0.05" euler="0 90 0">
        <joint name="wrl" type="hinge" axis="0 1 0" frictionloss="{WHEEL_FRI}"/>
        <geom name="wrl_g" type="cylinder" size="0.05 0.025" mass="0.08" friction="{WHEEL_FRI}"/>
      </body>
      <body name="wheel_rr" pos="-0.18 -0.15 0.05" euler="0 90 0">
        <joint name="wrr" type="hinge" axis="0 1 0" frictionloss="{WHEEL_FRI}"/>
        <geom name="wrr_g" type="cylinder" size="0.05 0.025" mass="0.08" friction="{WHEEL_FRI}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="L" joint="wfl" gear="{GEAR}"/>
    <motor name="R" joint="wfr" gear="{GEAR}"/>
  </actuator>
</mujoco>
"""


def build_model(mass_mult=1.0, friction_mult=1.0, gear_mult=1.0):
    """用域随机化参数构造 MuJoCo 小车模型并返回 data 与 body 速度读取句柄。
    参数倍率：mass_mult 质量、friction_mult 摩擦(阻力)、gear_mult 电机增益。"""
    xml = MUJOCO_XML.replace("{MASS}", f"{0.8 * mass_mult:g}") \
                    .replace("{BODY_FRI}", f"{0.5 * friction_mult:g}") \
                    .replace("{WHEEL_FRI}", f"{0.03 * friction_mult:g}") \
                    .replace("{GEAR}", f"{60 * gear_mult:g}")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    return model, data


def run_mujoco_episode(g, mass_mult, friction_mult, gear_mult, T=6.0):
    """用前馈增益 g 驱动 MuJoCo 小车，跟踪参考速度剖面，返回速度 RMSE。"""
    model, data = build_model(mass_mult, friction_mult, gear_mult)
    dt = model.opt.timestep
    speeds = []
    refs = []
    t = 0.0
    # 用 domain_rand_core 的参考速度剖面（匀速/加速/减速）
    while t <= T:
        v_ref = ref_speed(t)
        u = float(np.clip(g * v_ref, 0.0, 2.0))
        data.ctrl[0] = u
        data.ctrl[1] = u                     # 左右轮同力 → 直线前进
        mujoco.mj_step(model, data)
        vx = float(data.qvel[0])             # freejoint 线速度 x（车体前进方向）
        speeds.append(vx)
        refs.append(v_ref)
        t += dt
    return float(np.sqrt(np.mean((np.array(speeds) - np.array(refs)) ** 2)))


def main():
    # 域随机化采样器：仿真标称 vs 真实分布（sim-to-real gap）
    sampler = DomainRandomizer(seed=0)
    # 标称标定：用『仿真标称参数』在 MuJoCo 里实测稳态速度-指令映射
    # 前馈稳态 v = gear·u / friction，故 g_nom ≈ c/k（与 domain_rand_core 一致）
    g_nom = calibrate_nominal(sampler)
    g_dr = calibrate_domain_randomized(sampler, n_samples=60)

    print(f"仿真标称 g_nom = {g_nom:.3f}   域随机化标定 g_dr = {g_dr:.3f}")
    print("在真实参数（电机弱、阻力大）下于 MuJoCo 中评估两种标定：\n")

    # 真实分布采样测试集（部署参数落在分布内）
    test_params = [('仿真标称点', sampler.nominal())] + \
                  [(f"真实部署{i}", sampler.sample()) for i in range(8)]
    # 把 (m,k,c) 转成 MuJoCo 倍率（相对仿真标称 k0/c0）
    k0, c0 = sampler.k0, sampler.c0
    total_n = total_d = 0.0
    wins = 0
    for label, (m, k, c) in test_params:
        # m 直接作为质量倍率基准，k/c 相对标称作为电机/摩擦倍率
        gear_mult = k / k0
        friction_mult = c / c0
        mass_mult = m / 1.0
        rmse_n = run_mujoco_episode(g_nom, mass_mult, friction_mult, gear_mult)
        rmse_d = run_mujoco_episode(g_dr, mass_mult, friction_mult, gear_mult)
        total_n += rmse_n; total_d += rmse_d
        wins += rmse_d < rmse_n
        print(f"  {label:14s} k×{gear_mult:.2f} c×{friction_mult:.2f}  "
              f"标称RMSE={rmse_n:.3f}  DR-RMSE={rmse_d:.3f}  "
              f"胜={'DR' if rmse_d < rmse_n else '标称'}")
    n = len(test_params)
    print(f"\n平均速度RMSE  标称={total_n / n:.3f}  DR={total_d / n:.3f}"
          f"   鲁棒性提升={100 * (1 - total_d / total_n):.0f}%   DR胜{wins}/{n}")

    # 与 domain_rand_core 轻量模型做一致性对照（同一组测试参数）
    rows = evaluate(g_nom, g_dr, test_params)
    ln = np.mean([r[1] for r in rows]); ld = np.mean([r[2] for r in rows])
    print(f"[对照·轻量模型] 标称={ln:.3f}  DR={ld:.3f}  提升={100 * (1 - ld / ln):.0f}%")


if __name__ == '__main__':
    main()
