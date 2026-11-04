"""六个递进实验（L1-L6）。每个实验返回指标字典，可单独调用。"""

from __future__ import annotations

import os

import numpy as np
import mujoco
from PIL import Image

from .core import (
    DT, F_MAX, L0_MUS, R_HIP, R_KNEE, SJ_F_MAX, SJ_L0, SJ_R, SJ_COM_X,
    HillMuscle, Leg, joint_pd, allocate_pair, solve_joint_qp,
    single_joint_xml, two_joint_xml, legs_xml,
)


# ---------------- 单关节（L1-L3） ----------------

def _sj_state(theta: float, qvel: float):
    """解析肌肉学：屈肌随 +θ 缩短。"""
    L = np.array([SJ_L0 - SJ_R * theta, SJ_L0 + SJ_R * theta])
    Ldot = np.array([-SJ_R * qvel, +SJ_R * qvel])
    return L, Ldot


def _sj_torque(F):
    return SJ_R * (F[0] - F[1])


def run_l1(rec=None):
    """单条肌肉阶跃：肌肉能不能驱动关节。"""
    m = mujoco.MjModel.from_xml_string(single_joint_xml())
    d = mujoco.MjData(m)
    mus = [HillMuscle(SJ_L0 * 1.10, SJ_F_MAX), HillMuscle(SJ_L0 * 1.10, SJ_F_MAX)]
    ts, ang, frc = [], [], []
    for i in range(int(2.5 / DT)):
        t = i * DT
        L, Ld = _sj_state(float(d.qpos[0]), float(d.qvel[0]))
        u = np.array([0.30 if 0.4 <= t <= 1.0 else 0.05, 0.05])
        for j in (0, 1):
            mus[j].update_activation(u[j], DT)
        F = np.array([mus[0].force(L[0], Ld[0]), mus[1].force(L[1], Ld[1])])
        d.qfrc_applied[0] = _sj_torque(F)
        mujoco.mj_step(m, d)
        if rec:
            rec.maybe(d, i)
        ts.append(t); ang.append(float(d.qpos[0])); frc.append(float(F[0]))
    ang = np.array(ang)
    return {"peak_angle": float(ang.max()), "final_angle": float(ang[-1]),
            "peak_force": float(max(frc))}


def run_l2(rec=None):
    """拮抗肌对 + PD：能不能控到目标角。"""
    m = mujoco.MjModel.from_xml_string(single_joint_xml())
    rows = []
    for tgt in (0.0, 0.30, -0.30):
        d = mujoco.MjData(m)
        mus = [HillMuscle(SJ_L0 * 1.10, SJ_F_MAX), HillMuscle(SJ_L0 * 1.10, SJ_F_MAX)]
        hist = []
        for i in range(int(1.5 / DT)):
            L, Ld = _sj_state(float(d.qpos[0]), float(d.qvel[0]))
            tau = 9.0 * (tgt - d.qpos[0]) - 0.9 * d.qvel[0]
            a = allocate_pair(tau, SJ_R, SJ_F_MAX)
            for j in (0, 1):
                mus[j].update_activation(a[j], DT)
            F = np.array([mus[0].force(L[0], Ld[0]), mus[1].force(L[1], Ld[1])])
            d.qfrc_applied[0] = _sj_torque(F)
            mujoco.mj_step(m, d)
            if rec:
                rec.maybe(d, i)
            hist.append((float(d.qpos[0]), float(a.max())))
        h = np.array(hist)
        rows.append({"target": tgt, "final": float(h[-50:, 0].mean()),
                     "max_act": float(h[:, 1].max())})
    return {"targets": rows}


def run_l3():
    """共激活 -> 刚度：解析与数值互验。"""
    def static_torque(theta, base, l_opt):
        L, _ = _sj_state(theta, 0.0)
        F = []
        for j in (0, 1):
            mu = HillMuscle(l_opt, SJ_F_MAX)
            mu.a = base
            F.append(mu.force(L[j], 0.0))
        return _sj_torque(F)

    h = 1e-4
    rows = []
    for ratio, tag in ((1.10, "ascending"), (0.90, "descending")):
        l_opt = SJ_L0 * ratio
        ln = SJ_L0 / l_opt
        for base in (0.02, 0.10, 0.20, 0.40):
            k_num = -(static_torque(+h, base, l_opt) - static_torque(-h, base, l_opt)) / (2 * h)
            k_ana = (SJ_R ** 2 * 2.0 * SJ_F_MAX / l_opt
                     * (base * HillMuscle.fl_prime(ln) + HillMuscle.fp_prime(ln)))
            rows.append({"limb": tag, "base": base,
                         "stiffness_num": k_num, "stiffness_ana": k_ana})
    return {"sweep": rows}


# ---------------- 两关节（L4） ----------------

_MONO = {"hip_flex": (R_HIP, 0.0), "hip_ext": (-R_HIP, 0.0),
         "knee_ext": (0.0, -R_KNEE), "knee_flex": (0.0, R_KNEE)}
_BIA = {"hip_flex": (R_HIP, 0.0), "hip_ext": (-R_HIP, 0.0),
        "knee_ext": (0.0, -R_KNEE), "hamstring": (-R_HIP, R_KNEE)}


def _arm_matrix(model_dict):
    names = list(model_dict)
    return names, np.array([model_dict[n] for n in names]).T


def _simulate_two_joint(model_dict, mode, seconds=2.5, kp=5.0, kd=0.45):
    m = mujoco.MjModel.from_xml_string(two_joint_xml())
    d = mujoco.MjData(m)
    names, A = _arm_matrix(model_dict)
    muscles = [HillMuscle(L0_MUS * 1.10) for _ in names]
    knee_only = [i for i, n in enumerate(names) if n == "hamstring"]
    log = []

    for i in range(int(seconds / DT)):
        t = i * DT
        th = d.qpos.copy(); w = d.qvel.copy()
        knee = 0.0 if t < 0.4 else (0.9 if t > 1.0 else 0.9 * (t - 0.4) / 0.6)
        ff = np.array([d.qfrc_bias[m.joint("hip").dofadr[0]],
                       d.qfrc_bias[m.joint("knee").dofadr[0]]])
        tau = np.array([kp * (0.0 - th[0]) - kd * w[0],
                        kp * (knee - th[1]) - kd * w[1]]) + ff
        if mode == "qp":
            a = solve_joint_qp(A * F_MAX, tau)
        else:                                   # naive：忽略腘绳肌的髋分量
            A_eff = A.copy()
            for j in knee_only:
                A_eff[0, j] = 0.0
            a = solve_joint_qp(A_eff * F_MAX, tau)
        a = np.maximum(a, 0.02)

        L = L0_MUS - A.T @ th
        Ld = -A.T @ w
        for j in range(len(names)):
            muscles[j].update_activation(a[j], DT)
        F = np.array([muscles[j].force(L[j], Ld[j]) for j in range(len(names))])
        d.qfrc_applied[:] = A @ F
        mujoco.mj_step(m, d)
        log.append((t, float(d.qpos[0]), float(d.qpos[1]), *[float(x) for x in a]))

    arr = np.array(log)
    tail = arr[arr[:, 0] > seconds - 0.5]
    moving = arr[(arr[:, 0] > 0.4) & (arr[:, 0] < 2.2)]
    return {
        "hip_rms": float(np.sqrt(np.mean(tail[:, 1] ** 2))),
        "hip_max": float(np.max(np.abs(moving[:, 1]))),
        "knee_rms": float(np.sqrt(np.mean((tail[:, 2] - 0.9) ** 2))),
        "act_cost": float(np.mean(np.sum(arr[:, 3:], 1) ** 2)),
        "names": names,
    }


def run_l4():
    """双关节肌：按关节拆开分配 vs 联合 QP。"""
    return {
        "A_mono_qp": _simulate_two_joint(_MONO, "qp"),
        "B_bia_naive": _simulate_two_joint(_BIA, "naive"),
        "C_bia_qp": _simulate_two_joint(_BIA, "qp"),
    }


# ---------------- 负重（L5-L6） ----------------

def _simulate_legs(nlegs, seconds, targets, gscale, rec=None, kp=90.0, kd=9.0):
    m = mujoco.MjModel.from_xml_string(legs_xml(nlegs, gscale))
    d = mujoco.MjData(m)
    sides = ["L"] if nlegs == 1 else ["L", "R"]
    for s in sides:
        d.joint(f"hip_{s}").qpos[0] = -0.12
        d.joint(f"knee_{s}").qpos[0] = 0.25
    mujoco.mj_forward(m, d)

    legs = {s: Leg() for s in sides}
    rows = []
    for i in range(int(seconds / DT)):
        t = i * DT
        umax = 0.0
        for s in sides:
            q = np.array([d.joint(f"hip_{s}").qpos[0], d.joint(f"knee_{s}").qpos[0]])
            qd = np.array([d.joint(f"hip_{s}").qvel[0], d.joint(f"knee_{s}").qvel[0]])
            ff = np.array([d.qfrc_bias[m.joint(f"hip_{s}").dofadr[0]],
                           d.qfrc_bias[m.joint(f"knee_{s}").dofadr[0]]]) / nlegs
            tau = joint_pd(q, qd, targets(s, t), np.zeros(2), kp, kd) + ff
            u = legs[s].activations_for(tau)
            umax = max(umax, float(u.max()))
            L = legs[s].lengths(q)
            Ld = -legs[s].arms @ qd
            for j in range(4):
                legs[s].muscles[j].update_activation(u[j], DT)
            F = np.array([legs[s].muscles[j].force(L[j], Ld[j]) for j in range(4)])
            idx = [m.joint(f"hip_{s}").dofadr[0], m.joint(f"knee_{s}").dofadr[0]]
            d.qfrc_applied[idx] = legs[s].torques(F)
        mujoco.mj_step(m, d)
        if rec:
            rec.maybe(d, i)
        rows.append((t, float(d.joint("lift").qpos[0]),
                     float(d.geom(f"foot_g_{sides[0]}").xpos[2]),
                     float(d.joint(f"knee_{sides[0]}").qpos[0]), umax))
    return np.array(rows)


def run_l5(rec=None):
    """单腿负重站立（100% 重力）。"""
    rows = _simulate_legs(1, 4.0, lambda s, t: np.array([-0.12, 0.25]), 1.0, rec)
    z = rows[rows[:, 0] > 2.5, 1]
    return {"pelvis_z": float(z.mean()), "pelvis_z_std": float(z.std()),
            "knee": float(rows[-1, 3]), "max_act": float(rows[-1, 4])}


def run_l6(rec=None):
    """双腿原地踏步（35% 体重支撑）。"""
    T = 1.2

    def targets(s, t):
        phase = (t % T) / T
        swing = (phase < 0.5) if s == "L" else (phase >= 0.5)
        if swing:
            u = (phase % 0.5) / 0.5
            lift = np.sin(np.pi * u) ** 2
            return np.array([0.45 * lift - 0.12, 0.95 * lift + 0.25])
        return np.array([-0.12, 0.25])

    rows = _simulate_legs(2, 6.0, targets, 0.35, rec)
    z = rows[rows[:, 0] > 2.0, 1]
    foot = rows[:, 2]
    return {"pelvis_z": float(z.mean()), "pelvis_z_std": float(z.std()),
            "foot_clearance": float(foot.max() - foot.min()),
            "max_act": float(rows[-1, 4])}


# ---------------- 录制 ----------------

class Recorder:
    """MuJoCo 离屏渲染 -> GIF。渲染器在拿到第一帧时才创建（懒初始化）。"""

    def __init__(self, every=20, width=640, height=480,
                 lookat=(0.0, 0.0, -0.10), distance=0.8, elevation=-8.0, azimuth=140.0):
        self.renderer = None
        self.width, self.height = width, height
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.lookat[:] = lookat
        self.cam.distance = distance
        self.cam.elevation = elevation
        self.cam.azimuth = azimuth
        self.every = every
        self.frames = []

    def maybe(self, d, i):
        if i % self.every != 0:
            return
        if self.renderer is None:
            self.renderer = mujoco.Renderer(d.model, self.height, self.width)
        self.renderer.update_scene(d, camera=self.cam)
        self.frames.append(self.renderer.render().copy())

    def save(self, path, fps=25, scale=0.62):
        if not self.frames:
            raise RuntimeError("没有录制到任何帧")
        imgs = [Image.fromarray(f).resize(
            (int(f.shape[1] * scale), int(f.shape[0] * scale)), Image.LANCZOS)
            for f in self.frames]
        imgs[0].save(path, save_all=True, append_images=imgs[1:],
                     duration=int(1000 / fps), loop=0, optimize=True, disposal=2)
        if self.renderer is not None:
            self.renderer.close()
        return os.path.getsize(path)


