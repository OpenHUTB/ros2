"""肌肉驱动夹爪：核心模型（Hill 肌肉 + 两指 + 方块）。"""

from __future__ import annotations

import numpy as np

DT = 0.0005
F_MAX = 25.0          # 单条手指肌肉最大等长力 (N)
L0_MUS = 0.06         # 肌肉参考长度 (m)
SPRING = 800.0        # 手指回位弹簧刚度和 (N/m)

OBJ_HALF = 0.015      # 方块半边长 (m)
OBJ_MASS = 0.03       # 方块质量 (kg)
FINGER_MASS = 0.02

TABLE_Z = -0.060


class HillMuscle:
    def __init__(self, l_opt=L0_MUS * 1.10, f_max=F_MAX):
        self.l_opt = float(l_opt)
        self.f_max = float(f_max)
        self.v_max = 10.0 * self.l_opt
        self.a = 0.0

    @staticmethod
    def fl(ln):
        return float(np.exp(-((ln - 1.0) / 0.45) ** 2))

    @staticmethod
    def fp(ln):
        if ln <= 1.0:
            return 0.0
        return float((np.exp(5.0 * (ln - 1.0)) - 1.0) / (np.exp(5.0 * 0.6) - 1.0))

    @staticmethod
    def fv(vn):
        if vn >= 0:
            return float(0.3125 / (vn + 0.25) - 0.25)
        x = min(1.0, -vn)
        return float(1.0 + 0.4 * x / (0.25 + x))

    def update_activation(self, u, dt, tau_act=0.01, tau_deact=0.04):
        tau = tau_act if u > self.a else tau_deact
        self.a = float(np.clip(self.a + (u - self.a) * min(1.0, dt / tau), 0.0, 1.0))
        return self.a

    def force(self, length, length_dot):
        ln = length / self.l_opt
        vn = -length_dot / self.v_max
        return self.f_max * (self.a * self.fl(ln) * self.fv(vn) + self.fp(ln))


def gripper_xml(friction=1.5, obj_mass=OBJ_MASS, spring=SPRING):
    return f"""
<mujoco model="muscle_gripper">
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
    <geom name="table" type="plane" size="1.5 1.5 0.01" pos="0 0 {TABLE_Z}"
          material="grid" friction="{friction} 0.05 0.005"/>
    <body name="palm" pos="0 0 0">
      <joint name="pz" type="slide" axis="0 0 1" range="0 0.30" damping="40"/>
      <geom type="box" size="0.020 0.035 0.006" mass="0.4"
            rgba="0.40 0.40 0.45 1" contype="0" conaffinity="0"/>
      <body name="fl" pos="0 0.030 -0.026">
        <joint name="flj" type="slide" axis="0 -1 0" range="0 0.030"
               damping="4" stiffness="{spring}" springref="0"/>
        <geom type="box" size="0.008 0.006 0.022" mass="{FINGER_MASS}"
              friction="{friction} 0.05 0.005" solref="0.02 1"
              rgba="0.25 0.75 0.35 1"/>
      </body>
      <body name="fr" pos="0 -0.030 -0.026">
        <joint name="frj" type="slide" axis="0 1 0" range="0 0.030"
               damping="4" stiffness="{spring}" springref="0"/>
        <geom type="box" size="0.008 0.006 0.022" mass="{FINGER_MASS}"
              friction="{friction} 0.05 0.005" solref="0.02 1"
              rgba="0.25 0.75 0.35 1"/>
      </body>
    </body>
    <body name="obj" pos="0 0 {TABLE_Z + OBJ_HALF}">
      <freejoint name="objfree"/>
      <geom type="box" size="{OBJ_HALF} {OBJ_HALF} {OBJ_HALF}" mass="{obj_mass}"
            friction="{friction} 0.05 0.005" solref="0.02 1"
            rgba="0.90 0.25 0.25 1"/>
    </body>
  </worldbody>
  <actuator>
    <position joint="pz" kp="3000" kv="200" ctrlrange="0 0.30"/>
  </actuator>
</mujoco>
"""


def finger_muscle_length(q):
    """肌肉长度随手指合拢而缩短（绕指根的等效肌腱）。"""
    return L0_MUS - q


def muscle_force(muscle, q, qd):
    return muscle.force(finger_muscle_length(q), -qd)
