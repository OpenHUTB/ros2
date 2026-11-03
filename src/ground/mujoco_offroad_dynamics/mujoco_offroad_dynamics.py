#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mujoco_offroad_dynamics.py —— 车辆动力学 · 越野地形 MuJoCo 渲染版
================================================================================
在 MuJoCo 单轮车（低摩擦雪地路面）上验证『ABS 制动 vs 无 ABS 锁死』：
  - 无 ABS：制动扭矩直接拉满 → 车轮锁死（λ≈1）→ 轮胎在滑动摩擦下打滑滑行，
    制动距离长；
  - ABS：实时按滑移率 λ 调节制动扭矩，把 λ 控制在峰值摩擦附近（λ≈0.2）→
    始终使用更大可用摩擦 → 制动距离短、停车稳定。
用 MuJoCo 真实接触/摩擦动力学测量两车的滑行距离与用时，并与 offroad_core
轻量模型做一致性对照。

核心滑移率-摩擦模型与 2D 版共用 offroad_core.py；本文件只是把被控对象换成
MuJoCo 真实动力学渲染（本机安装 mujoco 后运行）。

运行:
  python3 mujoco_offroad_dynamics.py
说明：本项目为课程/学习用途的仿真代码，不含真实道路行驶与真车控制，遵循开源许可。
"""

import numpy as np
import mujoco

from offroad_core import ABS as ABS_Core, run_brake, OffroadTerrain

# 单轮车 + 低摩擦雪地路面（滑动摩擦 ≈ 0.18，峰值摩擦由接触摩擦决定）
MUJOCO_XML = r"""
<mujoco model="offroad_abs_car">
  <option timestep="0.002" integrator="Euler"/>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="snow" type="plane" size="20 10 0.1" friction="0.55 0.18 0.01"/>
    <body name="chassis" pos="0 0 0.6">
      <freejoint name="root"/>
      <geom name="body" type="box" size="0.5 0.35 0.3" mass="1495"/>
      <body name="wheel" pos="0 0 0.5" euler="0 90 0">
        <joint name="wheel_j" type="hinge" axis="0 1 0" frictionloss="0.3"/>
        <geom name="wheel_g" type="cylinder" size="0.3 0.09" mass="5" friction="0.55 0.18 0.01"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="brake" joint="wheel_j" gear="1"/>
  </actuator>
</mujoco>
"""


def build_model():
    model = mujoco.MjModel.from_xml_string(MUJOCO_XML)
    data = mujoco.MjData(model)
    # 定位 qvel 索引：freejoint(6) + wheel hinge(1)
    self_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    wheel_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_j")
    vx = model.jnt_dofadr[self_idx]          # 车体 x 线速度
    wq = model.jnt_dofadr[wheel_idx]         # 轮角速度
    return model, data, vx, wq


def run_mujoco_episode(use_abs, v0=30.0, r=0.3, T_max=3e4):
    """MuJoCo 单轮车雪地紧急制动，返回 (停止距离, 用时, 平均λ)。"""
    model, data, vx, wq = build_model()
    dt = model.opt.timestep
    # 初始车速与轮速（纯滚动）
    data.qvel[vx] = v0
    data.qvel[wq] = v0 / r
    abs_ctrl = ABS_Core() if use_abs else None
    T_brake = T_max if not use_abs else 0.0
    s0 = data.qpos[0]
    t = 0.0
    lams = []
    while t < 12.0:
        v = float(data.qvel[vx])
        w = float(data.qvel[wq])
        if v <= 0.05:
            break
        lam = float(np.clip((v - w * r) / max(v, 1e-3), 0.0, 1.0))
        if use_abs:
            T_brake = abs_ctrl.control(lam)
        data.ctrl[0] = -T_brake
        mujoco.mj_step(model, data)
        lams.append(lam)
        t += dt
    dist = data.qpos[0] - s0
    return dist, t, (float(np.mean(lams)) if lams else 0.0)


def main():
    # MuJoCo 雪地（低摩擦）渲染版
    print("MuJoCo 雪地路面（低摩擦）紧急制动，v0=30 m/s：")
    d_no, t_no, l_no = run_mujoco_episode(use_abs=False)
    d_ab, t_ab, l_ab = run_mujoco_episode(use_abs=True)
    print(f"  无ABS: 距离={d_no:.1f} m  用时={t_no:.1f}s  平均λ={l_no:.2f}(轮锁死打滑)")
    print(f"  ABS  : 距离={d_ab:.1f} m  用时={t_ab:.1f}s  平均λ={l_ab:.2f}(峰值摩擦附近)")
    if d_no > 0:
        print(f"  ABS 缩短制动距离 ≈ {100 * (1 - d_ab / d_no):.0f}%")

    # 与 offroad_core 轻量模型一致性对照（同一雪地 μ_peak≈0.55）
    terr = OffroadTerrain()
    # 用一个仅含低摩擦的对照地形
    class Snow:
        def at(self, s):
            return 0.55, 0.0
    veh = dict(m=1500, r=0.3, inertia=1.5, t_max=1.2e4)
    s_no, t_no2, _ = run_brake(veh, Snow(), use_abs=False, v0=30.0)
    s_ab, t_ab2, _ = run_brake(veh, Snow(), use_abs=True, v0=30.0)
    print(f"[对照·轻量] 无ABS={s_no:.1f}m  ABS={s_ab:.1f}m  缩短={100 * (1 - s_ab / s_no):.0f}%")


if __name__ == '__main__':
    main()
