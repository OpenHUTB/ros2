"""G5 抓持力闭环、G6 滑移反射、G7 负载扫描（追加到 runs.py 末尾）。"""

from __future__ import annotations

import mujoco
import numpy as np

from .core import DT, SPRING, HillMuscle, gripper_xml, muscle_force

LIFT_TARGET = 0.10
CLOSE_T = 1.5
LIFT_T = 4.0


def _fresh(friction=1.5, obj_mass=None):
    kw = {"friction": friction}
    if obj_mass is not None:
        kw["obj_mass"] = obj_mass
    m = mujoco.MjModel.from_xml_string(gripper_xml(**kw))
    d = mujoco.MjData(m)
    return m, d


def _grip_force_estimate(muscle, q, qd):
    """准静态估计：手指受力平衡时，接触法向力 = 肌肉力 − 弹簧回位力。"""
    return muscle_force(muscle, q, qd) - SPRING * q


def run_g5(targets=(2.0, 4.0, 8.0)):
    """G5 抓持力闭环：PI 调激活，跟踪目标夹持力。"""
    rows = []
    for target in targets:
        m, d = _fresh()
        dfl = m.joint("flj").dofadr[0]
        mus = HillMuscle()
        u = 0.30
        ki = 0.6
        hist = []
        t90 = None

        for i in range(int(3.0 / DT)):
            t = i * DT
            q = float(d.qpos[m.joint("flj").qposadr[0]])
            v = float(d.qvel[dfl])
            f_est = _grip_force_estimate(mus, q, v)
            u = float(np.clip(u + ki * (target - f_est) * DT, 0.0, 1.0))
            mus.update_activation(u, DT)
            qr = float(d.qpos[m.joint("frj").qposadr[0]])
            vr = float(d.qvel[m.joint("frj").dofadr[0]])
            d.qfrc_applied[dfl] = muscle_force(mus, q, v)
            d.qfrc_applied[m.joint("frj").dofadr[0]] = muscle_force(mus, qr, vr)
            mujoco.mj_step(m, d)
            hist.append(f_est)
            if t90 is None and f_est >= 0.9 * target:
                t90 = t

        steady = float(np.mean(hist[-int(0.5 / DT):]))
        rows.append({
            "target_n": target,
            "steady_n": steady,
            "steady_err_n": steady - target,
            "rise_time_s": t90 if t90 is not None else -1.0,
            "final_activation": u,
        })
    return {"levels": rows}


def run_g6(obj_mass=0.04, u0=0.33):
    """G6 滑移反射：检测到方块下滑就加力，对比固定激活。"""
    out = {}

    def sim(with_reflex: bool):
        m, d = _fresh(obj_mass=obj_mass)
        qa = m.jnt_qposadr[m.joint("objfree").id]
        dfl = m.joint("flj").dofadr[0]
        dfr = m.joint("frj").dofadr[0]
        mus_l, mus_r = HillMuscle(), HillMuscle()
        z_start = float(d.qpos[qa + 2])
        peak = 0.0
        u = u0                       # 起始激活：刚好在打滑边缘
        reflex_steps = 0        # 触发加力的**时间步**数（每步 +0.01 激活）
        lift_ref = 0.0

        for i in range(int(6.0 / DT)):
            t = i * DT
            d.ctrl[0] = 0.0 if t < CLOSE_T else LIFT_TARGET * min(1.0, (t - CLOSE_T) / LIFT_T)

            if with_reflex and t > CLOSE_T + 0.2:
                palm_z = float(d.qpos[m.joint("pz").qposadr[0]])
                obj_z = float(d.qpos[qa + 2])
                # 手掌已抬起但方块没跟上 -> 判为滑移，立刻加力
                if (palm_z - lift_ref) > (obj_z - z_start) + 0.002:
                    u = float(np.clip(u + 0.01, 0.0, 1.0))
                    reflex_steps += 1
                    lift_ref = palm_z

            ql = float(d.qpos[m.joint("flj").qposadr[0]])
            qr = float(d.qpos[m.joint("frj").qposadr[0]])
            mus_l.update_activation(u, DT)
            mus_r.update_activation(u, DT)
            d.qfrc_applied[dfl] = muscle_force(mus_l, ql, float(d.qvel[dfl]))
            d.qfrc_applied[dfr] = muscle_force(mus_r, qr, float(d.qvel[dfr]))
            mujoco.mj_step(m, d)

            peak = max(peak, float(d.qpos[qa + 2]) - z_start)

        return {"lift_mm": peak * 1000, "final_activation": u,
                "reflex_steps": reflex_steps,
                "reflex_activation_gain": 0.01 * reflex_steps}

    out["no_reflex"] = sim(False)
    out["with_reflex"] = sim(True)
    return out


def run_g7(masses=(0.01, 0.03, 0.06)):
    """G7 负载扫描：不同质量方块所需的最小激活。"""
    rows = []
    for mass in masses:
        best = None
        for act in (0.20, 0.25, 0.28, 0.30, 0.32, 0.35, 0.38, 0.40, 0.45, 0.50):
            m, d = _fresh(obj_mass=mass)
            qa = m.jnt_qposadr[m.joint("objfree").id]
            dfl = m.joint("flj").dofadr[0]
            dfr = m.joint("frj").dofadr[0]
            mus_l, mus_r = HillMuscle(), HillMuscle()
            z0 = float(d.qpos[qa + 2])
            peak = 0.0
            for i in range(int(5.0 / DT)):
                t = i * DT
                d.ctrl[0] = 0.0 if t < CLOSE_T else LIFT_TARGET * min(1.0, (t - CLOSE_T) / LIFT_T)
                ql = float(d.qpos[m.joint("flj").qposadr[0]])
                qr = float(d.qpos[m.joint("frj").qposadr[0]])
                mus_l.update_activation(act, DT)
                mus_r.update_activation(act, DT)
                d.qfrc_applied[dfl] = muscle_force(mus_l, ql, float(d.qvel[dfl]))
                d.qfrc_applied[dfr] = muscle_force(mus_r, qr, float(d.qvel[dfr]))
                mujoco.mj_step(m, d)
                peak = max(peak, float(d.qpos[qa + 2]) - z0)
            if peak > 0.05:
                best = act
                break
        rows.append({"mass_g": mass * 1000, "min_activation": best,
                     "gripped": best is not None})
    return {"levels": rows}


def run_g8(activation=0.6):
    """G8 手臂-夹爪联合抓取：两连杆手臂下探、夹爪合拢、抬起。"""
    from . import arm_grasp

    z0, z1, lift = arm_grasp.run(activation=activation)
    return {"obj_start_z": z0, "obj_end_z": z1, "lift_mm": lift * 1000}
