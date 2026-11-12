# -*- coding: utf-8 -*-
"""MuJoCo 人形机器人基础场景（MJCF）.

结构刻意保持"教科书式"的简单：骨盆 + 躯干 + 头 + 两条腿（髋/膝/踝）+ 两条胳膊（肩），
一共 8 个可动关节 + 1 个自由关节。所有关节都是绕 X 轴的铰链（pitch），
站立姿态就是"所有关节角为 0"，方便初学者对照 qpos 与关节名。
"""
from __future__ import annotations

XML = """
<mujoco model="humanoid_basics">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" iterations="50"/>
  <default>
    <geom friction="1.0 0.01 0.001" solref="0.01 1" solimp="0.9 0.95 0.001"/>
    <joint damping="0.5" armature="0.02" range="-1.6 1.6" limited="true"/>
    <motor gear="1" ctrlrange="-200 200"/>
  </default>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" directional="true"/>
    <light pos="2 2 3" dir="-1 -1 -1" diffuse="0.4 0.4 0.4"/>
    <geom name="floor" type="plane" size="4 4 0.05" rgba="0.82 0.82 0.82 1"/>

    <body name="pelvis" pos="0 0 1.05">
      <freejoint name="root"/>
      <geom type="box" size="0.16 0.11 0.10" mass="6" rgba="0.25 0.45 0.75 1"/>

      <body name="torso" pos="0 0 0.22">
        <geom type="box" size="0.17 0.11 0.20" mass="8" rgba="0.30 0.55 0.85 1"/>
        <body name="head" pos="0 0 0.30">
          <geom type="sphere" size="0.10" mass="1.5" rgba="0.90 0.80 0.65 1"/>
        </body>

        <body name="arm_l" pos="0 0.21 0.14">
          <joint name="shoulder_l" axis="0 1 0"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.28" size="0.045" mass="1.2" rgba="0.85 0.35 0.30 1"/>
          <body name="forearm_l" pos="0 0 -0.28">
            <joint name="elbow_l" axis="0 1 0" range="-2.4 0" limited="true" damping="0.3"/>
            <geom type="capsule" fromto="0 0 0 0 0 -0.26" size="0.04" mass="0.9" rgba="0.85 0.45 0.35 1"/>
          </body>
        </body>
        <body name="arm_r" pos="0 -0.21 0.14">
          <joint name="shoulder_r" axis="0 1 0"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.28" size="0.045" mass="1.2" rgba="0.85 0.35 0.30 1"/>
          <body name="forearm_r" pos="0 0 -0.28">
            <joint name="elbow_r" axis="0 1 0" range="-2.4 0" limited="true" damping="0.3"/>
            <geom type="capsule" fromto="0 0 0 0 0 -0.26" size="0.04" mass="0.9" rgba="0.85 0.45 0.35 1"/>
          </body>
        </body>
      </body>

      <body name="leg_l" pos="0 0.09 -0.10">
        <joint name="hip_l" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.45" size="0.055" mass="3.0" rgba="0.25 0.60 0.40 1"/>
        <body name="shin_l" pos="0 0 -0.45">
          <joint name="knee_l" axis="0 1 0" range="0 1.6" limited="true"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.43" size="0.05" mass="2.4" rgba="0.30 0.70 0.45 1"/>
          <body name="foot_l" pos="0 0 -0.43">
            <joint name="ankle_l" axis="0 1 0" range="-0.7 0.7" limited="true"/>
            <geom type="box" size="0.15 0.11 0.03" pos="0.02 0 -0.03" mass="0.6" rgba="0.20 0.20 0.25 1"/>
          </body>
        </body>
      </body>

      <body name="leg_r" pos="0 -0.09 -0.10">
        <joint name="hip_r" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.45" size="0.055" mass="3.0" rgba="0.25 0.60 0.40 1"/>
        <body name="shin_r" pos="0 0 -0.45">
          <joint name="knee_r" axis="0 1 0" range="0 1.6" limited="true"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.43" size="0.05" mass="2.4" rgba="0.30 0.70 0.45 1"/>
          <body name="foot_r" pos="0 0 -0.43">
            <joint name="ankle_r" axis="0 1 0" range="-0.7 0.7" limited="true"/>
            <geom type="box" size="0.15 0.11 0.03" pos="0.02 0 -0.03" mass="0.6" rgba="0.20 0.20 0.25 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="m_shoulder_l" joint="shoulder_l"/>
    <motor name="m_elbow_l" joint="elbow_l"/>
    <motor name="m_shoulder_r" joint="shoulder_r"/>
    <motor name="m_elbow_r" joint="elbow_r"/>
    <motor name="m_hip_l" joint="hip_l"/>
    <motor name="m_knee_l" joint="knee_l"/>
    <motor name="m_ankle_l" joint="ankle_l"/>
    <motor name="m_hip_r" joint="hip_r"/>
    <motor name="m_knee_r" joint="knee_r"/>
    <motor name="m_ankle_r" joint="ankle_r"/>
  </actuator>
</mujoco>
"""

# 站立姿态：所有关节角为 0（直腿、手臂自然下垂）
JOINTS = (
    "shoulder_l", "elbow_l", "shoulder_r", "elbow_r",
    "hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r",
)
