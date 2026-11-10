"""手臂 + 肌肉驱动夹爪：在已验证的夹爪模型上加两连杆手臂。"""

from __future__ import annotations

import os

import mujoco
import numpy as np
from PIL import Image

from .core import DT, HillMuscle, muscle_force

ARM_TABLE_Z = -0.36
ARM_OBJ_HALF = 0.015
ARM_OBJ_Z = ARM_TABLE_Z + ARM_OBJ_HALF
GRASP_X = 0.15
HAND_GRASP_Z = ARM_OBJ_Z + 0.045      # 手部原点高度，使手指夹住方块中部

UPPER_L = 0.25
FORE_L = 0.25


# 手臂质量：上臂 / 前臂 / 手掌
def arm_gripper_xml(arm_mass=(2.0, 1.2, 0.2)):
    mu, mf, mh = arm_mass
    return f"""
<mujoco model="arm_muscle_gripper">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{DT}" integrator="implicitfast" gravity="0 0 -9.81"
          cone="elliptic" impratio="10"/>
  <visual>
    <global offwidth="640" offheight="480" fovy="45"/>
    <headlight ambient="0.45 0.45 0.5" diffuse="0.6 0.6 0.6"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.26 0.26 0.30"
             rgb2="0.17 0.17 0.20" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0.5 -0.8 1.2" dir="-0.2 0.4 -1"/>
    <geom name="table" type="plane" size="1.5 1.5 0.01" pos="0 0 {ARM_TABLE_Z}"
          material="grid" friction="1.5 0.05 0.005"/>
    <body name="torso" pos="0 0 0">
      <geom type="box" size="0.06 0.05 0.10" pos="0 0 0.05"
            rgba="0.55 0.57 0.62 1" mass="6" contype="0" conaffinity="0"/>
      <body name="upper" pos="0 0 0">
        <joint name="shoulder" type="hinge" axis="0 -1 0" range="-1.6 1.8"
               damping="0.20" armature="0.004"/>
        <geom type="capsule" size="0.026" fromto="0 0 0 0 0 -{UPPER_L}"
              mass="{mu}" rgba="0.20 0.55 0.95 1" contype="0" conaffinity="0"/>
        <body name="fore" pos="0 0 -{UPPER_L}">
          <joint name="elbow" type="hinge" axis="0 -1 0" range="0.0 2.4"
                 damping="0.15" armature="0.002"/>
          <geom type="capsule" size="0.023" fromto="0 0 0 0 0 -{FORE_L}"
                mass="{mf}" rgba="0.95 0.55 0.20 1" contype="0" conaffinity="0"/>
          <body name="hand" pos="0 0 -{FORE_L}">
            <joint name="wrist" type="hinge" axis="0 -1 0" range="-1.6 1.6"
                   damping="0.10" armature="0.001"/>
            <site name="ee" pos="0 0 0"/>
            <geom type="box" size="0.020 0.035 0.006" pos="0 0 -0.006"
                  mass="{mh}" rgba="0.40 0.40 0.45 1"
                  contype="0" conaffinity="0"/>
            <body name="finger_l" pos="0 0.030 -0.026">
              <joint name="flj" type="slide" axis="0 -1 0" range="0 0.030"
                     damping="4" stiffness="800" springref="0"/>
              <geom type="box" size="0.008 0.006 0.022" mass="0.02"
                    friction="1.5 0.05 0.005" solref="0.02 1"
                    rgba="0.25 0.75 0.35 1"/>
            </body>
            <body name="finger_r" pos="0 -0.030 -0.026">
              <joint name="frj" type="slide" axis="0 1 0" range="0 0.030"
                     damping="4" stiffness="800" springref="0"/>
              <geom type="box" size="0.008 0.006 0.022" mass="0.02"
                    friction="1.5 0.05 0.005" solref="0.02 1"
                    rgba="0.25 0.75 0.35 1"/>
            </body>
          </body>
        </body>
      </body>
    </body>
    <body name="obj" pos="{GRASP_X} 0 {ARM_OBJ_Z}">
      <freejoint name="objfree"/>
      <geom type="box" size="{ARM_OBJ_HALF} {ARM_OBJ_HALF} {ARM_OBJ_HALF}" mass="0.03"
            friction="1.5 0.05 0.005" solref="0.02 1" rgba="0.90 0.25 0.25 1"/>
    </body>
  </worldbody>
  <actuator>
    <position joint="wrist" kp="400" kv="40" ctrlrange="-1.6 1.6"/>
  </actuator>
</mujoco>
"""


def ik(x, z):
    """两连杆逆解：输入手部世界坐标 (x, z)，返回 [肩, 肘] 角。

    正解：ee_x = L1 sin(q0) + L2 sin(q0+q1)
          ee_z = -(L1 cos(q0) + L2 cos(q0+q1))
    """
    zz = -z
    r2 = x * x + zz * zz
    c1 = (r2 - UPPER_L ** 2 - FORE_L ** 2) / (2.0 * UPPER_L * FORE_L)
    q1 = float(np.arccos(max(-1.0, min(1.0, c1))))
    q0 = float(np.arctan2(x, zz)
               - np.arctan2(FORE_L * np.sin(q1), UPPER_L + FORE_L * np.cos(q1)))
    return np.array([q0, q1])


def run(activation=0.6, seconds=7.0, rec=None, verbose=False):
    """下降 → 合拢 → 抬起；返回 (方块起始高度, 结束高度, 抬升)。"""
    m = mujoco.MjModel.from_xml_string(arm_gripper_xml())
    d = mujoco.MjData(m)
    q0 = ik(GRASP_X, HAND_GRASP_Z)
    d.qpos[0], d.qpos[1] = q0[0], q0[1]
    d.qpos[2] = -(q0[0] + q0[1])
    mujoco.mj_forward(m, d)

    dfl = m.joint("flj").dofadr[0]
    dfr = m.joint("frj").dofadr[0]
    qa = m.jnt_qposadr[m.joint("objfree").id]
    mfl, mfr = HillMuscle(), HillMuscle()

    kp = np.array([220.0, 150.0])
    kd = np.array([14.0, 9.0])

    # 关键帧：(时刻, 手掌世界 z)
    way = [(0.0, -0.24), (1.2, -0.24), (2.6, HAND_GRASP_Z),
           (4.2, HAND_GRASP_Z), (7.0, -0.21), (7.4, -0.21)]

    def target_z(t):
        for k in range(len(way) - 1):
            t0, za = way[k]
            t1, zb = way[k + 1]
            if t <= t1:
                u = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return za + (zb - za) * max(0.0, min(1.0, u))
        return way[-1][1]

    z0 = None
    for i in range(int(seconds / DT)):
        t = i * DT
        hz = target_z(t)
        act = activation if t >= 2.9 else 0.0

        q = np.array([d.qpos[0], d.qpos[1]])
        qd = np.array([d.qvel[0], d.qvel[1]])
        q_des = ik(GRASP_X, hz)
        tau = kp * (q_des - q) - kd * qd + np.array([d.qfrc_bias[0], d.qfrc_bias[1]])
        d.qfrc_applied[0] = tau[0]
        d.qfrc_applied[1] = tau[1]
        d.ctrl[0] = -(q[0] + q[1])

        ql = float(d.qpos[m.joint("flj").qposadr[0]])
        qr = float(d.qpos[m.joint("frj").qposadr[0]])
        mfl.update_activation(act, DT)
        mfr.update_activation(act, DT)
        d.qfrc_applied[dfl] = muscle_force(mfl, ql, float(d.qvel[dfl]))
        d.qfrc_applied[dfr] = muscle_force(mfr, qr, float(d.qvel[dfr]))

        mujoco.mj_step(m, d)
        if rec:
            rec.maybe(d, i)
        if i == int(2.9 / DT):
            z0 = float(d.qpos[qa + 2])
        if verbose and i % int(0.5 / DT) == 0:
            print(f"  t={t:4.1f}s 手={d.site('ee').xpos[2]:+.3f} "
                  f"方块={d.qpos[qa + 2]:+.3f} 指={ql:.4f} 接触={d.ncon}")

    z1 = float(d.qpos[qa + 2])
    if z0 is None:
        z0 = z1
    return z0, z1, z1 - z0


class Rec:
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
            self.cam.lookat[:] = [0.12, 0.0, -0.30]
            self.cam.distance = 0.78
            self.cam.elevation = -14.0
            self.cam.azimuth = 148.0
        self.renderer.update_scene(d, camera=self.cam)
        self.frames.append(Image.fromarray(self.renderer.render().copy()))

    def save(self, path, fps=25, scale=0.62):
        imgs = [f.resize((int(f.width * scale), int(f.height * scale)), Image.LANCZOS)
                for f in self.frames]
        imgs[0].save(path, save_all=True, append_images=imgs[1:],
                     duration=int(1000 / fps), loop=0, optimize=False, disposal=1)
        if self.renderer:
            self.renderer.close()
        return os.path.getsize(path)
