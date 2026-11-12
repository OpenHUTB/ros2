# -*- coding: utf-8 -*-
"""平地环境 + CPG 基线 + 统一 rollout（所有对比同源）。"""

import numpy as np

import humanoid_ars_phase_walk.config as config

JOINTS = ("hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r")
OBS_DIM = 17
ACT_DIM = 6


def cpg(ph):
    """相位 -> 6 个关节目标角。实测能走 1.53 m / 4 s 且躯干直立。"""
    hl = config.A_HIP * np.sin(ph)
    hr = config.A_HIP * np.sin(ph + np.pi)
    kl = config.A_KNEE * max(0.0, np.sin(ph + config.KNEE_PHASE))
    kr = config.A_KNEE * max(0.0, np.sin(ph + np.pi + config.KNEE_PHASE))
    return np.array([hl, kl, -config.ANKLE_GAIN * (hl + kl),
                     hr, kr, -config.ANKLE_GAIN * (hr + kr)])


FLAT_XML = """<mujoco model="ars_flat">
  <compiler angle="radian" coordinate="local" inertiafromgeom="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual><global offwidth="480" offheight="270"/></visual>
  <default><geom contype="0" conaffinity="0" group="1"/></default>
  <worldbody>
    <light pos="0 -2 3" dir="0 0.4 -1" diffuse="0.7 0.7 0.7"/>
    <geom name="floor" type="plane" size="6 2 0.1" contype="1" conaffinity="1" group="0"/>
    <body name="torso" pos="0 0 0.80">
      <joint name="root_x" type="slide" axis="1 0 0"/>
      <joint name="root_z" type="slide" axis="0 0 1"/>
      <joint name="root_pitch" type="hinge" axis="0 1 0"/>
      <geom name="torso_geom" type="capsule" fromto="0 0 -0.10 0 0 0.30" size="0.11"
            mass="8.0" contype="1" conaffinity="1" group="0"/>
      <body name="thigh_l" pos="0 0.09 0">
        <joint name="hip_l" type="hinge" axis="0 1 0" range="-1.4 1.4"/>
        <geom name="thigh_geom_l" type="capsule" fromto="0 0 0 0 0 -0.40" size="0.045" mass="2.0"/>
        <body name="shank_l" pos="0 0 -0.40">
          <joint name="knee_l" type="hinge" axis="0 1 0" range="0 2.2"/>
          <geom name="shank_geom_l" type="capsule" fromto="0 0 0 0 0 -0.40" size="0.040" mass="2.0"/>
          <body name="foot_l" pos="0 0 -0.40">
            <joint name="ankle_l" type="hinge" axis="0 1 0" range="-0.9 0.9"/>
            <geom name="foot_geom_l" type="box" pos="0.08 0 -0.02" size="0.11 0.05 0.02"
                  mass="1.0" contype="1" conaffinity="1" group="0"/>
          </body>
        </body>
      </body>
      <body name="thigh_r" pos="0 -0.09 0">
        <joint name="hip_r" type="hinge" axis="0 1 0" range="-1.4 1.4"/>
        <geom name="thigh_geom_r" type="capsule" fromto="0 0 0 0 0 -0.40" size="0.045" mass="2.0"/>
        <body name="shank_r" pos="0 0 -0.40">
          <joint name="knee_r" type="hinge" axis="0 1 0" range="0 2.2"/>
          <geom name="shank_geom_r" type="capsule" fromto="0 0 0 0 0 -0.40" size="0.040" mass="2.0"/>
          <body name="foot_r" pos="0 0 -0.40">
            <joint name="ankle_r" type="hinge" axis="0 1 0" range="-0.9 0.9"/>
            <geom name="foot_geom_r" type="box" pos="0.08 0 -0.02" size="0.11 0.05 0.02"
                  mass="1.0" contype="1" conaffinity="1" group="0"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="hip_l"   joint="hip_l"   kp="400" kv="40" ctrlrange="-1.4 1.4"/>
    <position name="knee_l"  joint="knee_l"  kp="400" kv="40" ctrlrange="0 2.2"/>
    <position name="ankle_l" joint="ankle_l" kp="200" kv="20" ctrlrange="-0.9 0.9"/>
    <position name="hip_r"   joint="hip_r"   kp="400" kv="40" ctrlrange="-1.4 1.4"/>
    <position name="knee_r"  joint="knee_r"  kp="400" kv="40" ctrlrange="0 2.2"/>
    <position name="ankle_r" joint="ankle_r" kp="200" kv="20" ctrlrange="-0.9 0.9"/>
  </actuator>
</mujoco>
"""


def make_env(seed=config.SEED):
    """自带的平地场景（不依赖仓库里其它模块）。"""
    import mujoco
    rng = np.random.default_rng(seed)
    m = mujoco.MjModel.from_xml_string(FLAT_XML)
    d = mujoco.MjData(m)
    return mujoco, m, d, rng


def _idx(mj, m):
    nm = lambda n: int(mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n))
    q = {n: int(m.jnt_qposadr[nm(n)]) for n in ("root_x", "root_z", "root_pitch") + JOINTS}
    v = {n: int(m.jnt_dofadr[nm(n)]) for n in ("root_x", "root_z", "root_pitch") + JOINTS}
    return q, v


def rollout(env, policy=None, record=False):
    """跑一个回合。``policy(ph, obs) -> 6 维动作``；None 为纯 CPG。"""
    mj, m, d, rng = env
    mj.mj_resetData(m, d)
    q, v = _idx(mj, m)
    d.qpos[q["root_z"]] = 0.0
    for n in JOINTS:
        d.qpos[q[n]] = 0.0
    mj.mj_forward(m, d)
    x0 = float(d.qpos[q["root_x"]])
    sub = max(1, int(round(1.0 / config.CTRL_HZ / m.opt.timestep)))
    R = 0.0; pitches = []; speeds = []; X = []; Y = []
    energy = 0.0; steps = 0
    for k in range(int(config.EPISODE_T * config.CTRL_HZ)):
        ph = 2 * np.pi * config.FREQ * (k / config.CTRL_HZ)
        obs = np.array([np.sin(ph), np.cos(ph)]
                       + [float(d.qpos[q[n]]) for n in JOINTS]
                       + [float(d.qvel[v[n]]) for n in JOINTS]
                       + [float(d.qpos[q["root_pitch"]]),
                          float(d.qvel[v["root_pitch"]]), float(d.qvel[v["root_x"]])])
        target = cpg(ph)
        if record:
            X.append(obs); Y.append(target)
        ctrl = target if policy is None else np.asarray(policy(ph, obs), dtype=float)
        ctrl = np.clip(ctrl, -1.2, 1.2)
        d.ctrl[:] = ctrl
        for _ in range(sub):
            mj.mj_step(m, d)
        pitch = abs(float(d.qpos[q["root_pitch"]]))
        sp = float(d.qvel[v["root_x"]])
        pitches.append(pitch); speeds.append(sp)
        energy += config.W_EFFORT * float(np.sum(ctrl ** 2))
        R += (config.W_VEL * sp - config.W_PITCH * pitch + config.W_ALIVE
              - config.W_EFFORT * float(np.sum(ctrl ** 2)))
        steps = k + 1
        if pitch > config.FALL_PITCH or float(d.qpos[q["root_z"]]) < config.FALL_DZ:
            R -= 3.0
            break
    out = dict(dx=float(d.qpos[q["root_x"]]) - x0, steps=steps, reward=R,
               pitch_rms=float(np.sqrt(np.mean(np.square(pitches)))),
               speed=float(np.mean(speeds)), energy=energy)
    if record:
        out["X"] = np.asarray(X); out["Y"] = np.asarray(Y)
    return out