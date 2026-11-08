"""四组实验：肌肉激活 → 夹持力 → 抬升高度 / 打滑阈值。"""

from __future__ import annotations

import os

import mujoco
import numpy as np
from PIL import Image

from .core import DT, SPRING, HillMuscle, gripper_xml, muscle_force

LIFT_TARGET = 0.10      # 手掌抬升目标 (m)
CLOSE_T = 1.5           # 闭合时间 (s)
LIFT_T = 4.0            # 抬升时间 (s)


def _make(friction=1.5, obj_mass=None, spring=None):
    kw = {"friction": friction}
    if obj_mass is not None:
        kw["obj_mass"] = obj_mass
    if spring is not None:
        kw["spring"] = spring
    m = mujoco.MjModel.from_xml_string(gripper_xml(**kw))
    return m, mujoco.MjData(m)


def _simulate(activation, friction=1.5, obj_mass=None, rec=None, seconds=None,
              verbose=False):
    """闭合 → 抬升；返回 (方块高度序列, 是否夹住, 起始高度 z_start)。"""
    m, d = _make(friction=friction, obj_mass=obj_mass)
    qa = m.jnt_qposadr[m.joint("objfree").id]
    dfl = m.joint("flj").dofadr[0]
    dfr = m.joint("frj").dofadr[0]
    mfl = HillMuscle()
    mfr = HillMuscle()

    total = seconds if seconds is not None else CLOSE_T + LIFT_T + 0.5
    z_start = float(d.qpos[qa + 2])
    heights = []
    gripped = False

    for i in range(int(total / DT)):
        t = i * DT
        # 抬升目标：闭合结束后线性上升
        if t < CLOSE_T:
            d.ctrl[0] = 0.0
        else:
            u = min(1.0, (t - CLOSE_T) / LIFT_T)
            d.ctrl[0] = LIFT_TARGET * u

        qf = float(d.qpos[m.joint("flj").qposadr[0]])
        qfr = float(d.qpos[m.joint("frj").qposadr[0]])
        vf = float(d.qvel[dfl])
        vfr = float(d.qvel[dfr])

        mfl.update_activation(activation, DT)
        mfr.update_activation(activation, DT)
        d.qfrc_applied[dfl] = muscle_force(mfl, qf, vf)
        d.qfrc_applied[dfr] = muscle_force(mfr, qfr, vfr)

        mujoco.mj_step(m, d)
        if rec:
            rec.maybe(d, i)

        z = float(d.qpos[qa + 2])
        heights.append(z)
        if not gripped and z > z_start + 0.010:
            gripped = True

        if verbose and i % int(0.5 / DT) == 0:
            print(f"  t={t:4.1f}s 手掌={d.qpos[m.joint('pz').qposadr[0]]:+.3f} "
                  f"方块={z:+.3f} 指={qf:.4f} 接触={d.ncon}")

    return np.array(heights), gripped, z_start


def _finger_contact(m, d):
    """返回 (手指-方块接触点数, 接触法向力合计)，直接读 MuJoCo 接触力。

    手指与方块的 geom 未命名，因此按 **body 名** 匹配：
    finger_l / finger_r 与 obj 之间的接触才算夹持接触。
    """
    body_names = {}
    for i in range(m.nbody):
        body_names[i] = m.body(i).name

    def body_of(geom_id):
        return body_names.get(int(m.geom_bodyid[geom_id]), "")

    n = 0
    total = 0.0
    force6 = np.zeros(6)
    fingers = {"fl", "fr"}
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = body_of(c.geom1), body_of(c.geom2)
        if "obj" not in (b1, b2):
            continue
        if not (fingers & {b1, b2}):
            continue
        mujoco.mj_contactForce(m, d, i, force6)
        n += 1
        total += abs(float(force6[0]))     # 法向分量
    return n, total


def run_g1():
    """G1 肌肉激活 → 稳态夹持力（等长，不抬升）。

    夹持力取**手指与方块之间的真实接触法向力**（mj_contactForce），
    而不是肌力：肌力要先扣掉回位弹簧力才作用到方块上，
    且手指尚未接触方块时接触力为 0。
    双指对称驱动，与 G2/G3/G6/G7 口径一致。
    """
    rows = []
    for act in (0.05, 0.10, 0.20, 0.40, 0.80):
        m, d = _make()
        dfl = m.joint("flj").dofadr[0]
        dfr = m.joint("frj").dofadr[0]
        mus_l, mus_r = HillMuscle(), HillMuscle()
        for _ in range(int(1.2 / DT)):
            ql = float(d.qpos[m.joint("flj").qposadr[0]])
            qr = float(d.qpos[m.joint("frj").qposadr[0]])
            mus_l.update_activation(act, DT)
            mus_r.update_activation(act, DT)
            d.qfrc_applied[dfl] = muscle_force(mus_l, ql, float(d.qvel[dfl]))
            d.qfrc_applied[dfr] = muscle_force(mus_r, qr, float(d.qvel[dfr]))
            mujoco.mj_step(m, d)

        ql = float(d.qpos[m.joint("flj").qposadr[0]])
        f_mus = muscle_force(mus_l, ql, float(d.qvel[dfl]))
        f_spring = SPRING * ql
        n_con, f_con = _finger_contact(m, d)
        rows.append({"activation": act,
                     "grip_force_n": float(f_con),            # 真实接触法向力合计
                     "muscle_minus_spring_n": float(f_mus - f_spring),
                     "muscle_n": float(f_mus),
                     "spring_n": float(f_spring),
                     "contact_points": int(n_con),
                     "finger_travel_m": ql})
    return {"levels": rows}


def run_g2():
    """G2 夹持力/激活 → 最大抬升高度（滑脱即止）。"""
    rows = []
    for act in (0.05, 0.10, 0.20, 0.40, 0.80):
        h, gripped, z0 = _simulate(act)
        peak = float(h.max() - z0)
        rows.append({"activation": act, "lift_mm": peak * 1000,
                     "gripped": bool(gripped)})
    return {"levels": rows}


def run_g3():
    """G3 摩擦系数 × 激活 → 抬升高度，找打滑阈值。"""
    rows = []
    for mu in (0.3, 0.8, 1.5):
        for act in (0.10, 0.40):
            h, gripped, z0 = _simulate(act, friction=mu)
            rows.append({"friction": mu, "activation": act,
                         "lift_mm": float((h.max() - z0) * 1000),
                         "gripped": bool(gripped)})
    return {"levels": rows}


def run_g4():
    """G4 对照组：先夹住抬起，再放松肌肉，观察是否掉落。"""
    m, d = _make()
    qa = m.jnt_qposadr[m.joint("objfree").id]
    dfl = m.joint("flj").dofadr[0]
    dfr = m.joint("frj").dofadr[0]
    mfl, mfr = HillMuscle(), HillMuscle()
    z_start = float(d.qpos[qa + 2])
    drop_t = None
    peak = 0.0

    for i in range(int(6.0 / DT)):
        t = i * DT
        # 0-1.5 闭合；1.5-3.5 抬升；3.5 之后松开肌肉
        if t < CLOSE_T:
            d.ctrl[0] = 0.0
        else:
            d.ctrl[0] = LIFT_TARGET * min(1.0, (t - CLOSE_T) / 2.0)
        act = 0.6 if t < 3.5 else 0.0

        qf = float(d.qpos[m.joint("flj").qposadr[0]])
        qfr = float(d.qpos[m.joint("frj").qposadr[0]])
        mfl.update_activation(act, DT)
        mfr.update_activation(act, DT)
        d.qfrc_applied[dfl] = muscle_force(mfl, qf, float(d.qvel[dfl]))
        d.qfrc_applied[dfr] = muscle_force(mfr, qfr, float(d.qvel[dfr]))
        mujoco.mj_step(m, d)

        z = float(d.qpos[qa + 2])
        peak = max(peak, z - z_start)
        if drop_t is None and t > 3.5 and z < z_start + 0.005:
            drop_t = float(t - 3.5)

    return {"lift_mm": peak * 1000,
            "drop_after_release_s": drop_t if drop_t is not None else -1.0}


class Recorder:
    def __init__(self, every=40):
        self.every = every
        self.frames = []
        self.renderer = None

    def maybe(self, d, i):
        if i % self.every:
            return
        if self.renderer is None:
            self.renderer = mujoco.Renderer(d.model, 420, 560)
            self.cam = mujoco.MjvCamera()
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.cam.lookat[:] = [0.0, 0.0, -0.03]
            self.cam.distance = 0.34
            self.cam.elevation = -12.0
            self.cam.azimuth = 135.0
        self.renderer.update_scene(d, camera=self.cam)
        self.frames.append(Image.fromarray(self.renderer.render().copy()))

    def save(self, path, fps=25, scale=0.8):
        imgs = [f.resize((int(f.width * scale), int(f.height * scale)), Image.LANCZOS)
                for f in self.frames]
        imgs[0].save(path, save_all=True, append_images=imgs[1:],
                     duration=int(1000 / fps), loop=0, optimize=False, disposal=1)
        if self.renderer:
            self.renderer.close()
        return os.path.getsize(path)
