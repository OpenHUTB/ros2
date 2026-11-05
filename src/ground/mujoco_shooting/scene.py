# -*- coding: utf-8 -*-
"""MuJoCo scene (MJCF) for the shooting demo: car + turret + 6 targets + 12 balls."""
from __future__ import annotations

XML = """
<mujoco>
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" iterations="60"/>
  <default>
    <geom friction="1.0 0.005 0.0001" solref="0.02 1" solimp="0.9 0.95 0.001"/>
    <joint damping="0.05" armature="0.02"/>
  </default>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" directional="true"/>
    <light pos="-3 3 4" dir="1 -1 -1" diffuse="0.3 0.3 0.35"/>
    <geom name="floor" type="plane" size="8 8 0.05" rgba="0.75 0.75 0.75 1"/>
    <geom name="zone" type="box" pos="2.0 0 0.001" size="2.0 2.9 0.001" rgba="0.50 0.60 0.50 1"/>
    <geom name="line" type="box" pos="0.4 0 0.002" size="0.02 2.9 0.002" rgba="0.90 0.90 0.90 1"/>
    <body name="car" pos="0 0 0.05">
      <geom name="chassis" type="box" pos="0 0 0.04" size="0.28 0.16 0.035" rgba="0.20 0.45 0.85 1" mass="4.0"/>
      <body name="turret" pos="0.28 0 0.12">
        <joint name="turret_j" type="hinge" axis="0 0 1"/>
        <geom name="barrel" type="box" pos="0.15 0 0" size="0.15 0.03 0.03" rgba="0.25 0.25 0.25 1"/>
        <geom name="beam" type="cylinder" pos="2.0 0 0" size="0.012 1.7" euler="0 1.5707963 0" rgba="0.10 1.00 0.30 0.30" contype="0" conaffinity="0"/>
      </body>
      <body name="c_fl" pos="0.16 0.125 0">
        <joint name="c_fl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_fr" pos="0.16 -0.125 0">
        <joint name="c_fr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_rl" pos="-0.16 0.125 0">
        <joint name="c_rl" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
      <body name="c_rr" pos="-0.16 -0.125 0">
        <joint name="c_rr" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" friction="2.5 0.01 0.0001" size="0.05 0.03" euler="1.5707963 0 0" rgba="0.08 0.08 0.08 1" mass="0.2"/>
      </body>
    </body>
    <body name="t0" pos="1.55 -1.25 0">
      <joint name="tj0" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.85 0.25 0.25 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx0" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy0" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t1" pos="2.00 0.00 0">
      <joint name="tj1" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.95 0.60 0.15 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx1" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy1" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t2" pos="1.55 1.25 0">
      <joint name="tj2" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.90 0.85 0.20 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx2" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy2" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t3" pos="3.00 -1.35 0">
      <joint name="tj3" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.30 0.70 0.35 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx3" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy3" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t4" pos="3.40 0.00 0">
      <joint name="tj4" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.25 0.55 0.85 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx4" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy4" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="t5" pos="3.00 1.35 0">
      <joint name="tj5" type="hinge" axis="0 1 0" limited="true" range="-1.4 1.4"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.15 0.02" euler="0 1.5707963 0" rgba="0.60 0.35 0.80 1" mass="0.1"/>
      <geom type="cylinder" pos="0 0 0.25" size="0.05 0.021" euler="0 1.5707963 0" rgba="0.97 0.97 0.97 1" mass="0.01"/>
      <geom name="cx5" type="box" pos="0 0 0.45" size="0.13 0.016 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
      <geom name="cy5" type="box" pos="0 0 0.45" size="0.016 0.13 0.010" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <body name="ball0" pos="10 10 0.05">
      <freejoint name="bj0"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball1" pos="10 10.2 0.05">
      <freejoint name="bj1"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball2" pos="10 10.4 0.05">
      <freejoint name="bj2"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball3" pos="10 10.6 0.05">
      <freejoint name="bj3"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball4" pos="10 10.8 0.05">
      <freejoint name="bj4"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball5" pos="10 11.0 0.05">
      <freejoint name="bj5"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball6" pos="10 11.2 0.05">
      <freejoint name="bj6"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball7" pos="10 11.4 0.05">
      <freejoint name="bj7"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball8" pos="10 11.6 0.05">
      <freejoint name="bj8"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball9" pos="10 11.8 0.05">
      <freejoint name="bj9"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball10" pos="10 12.0 0.05">
      <freejoint name="bj10"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
    <body name="ball11" pos="10 12.2 0.05">
      <freejoint name="bj11"/>
      <geom type="sphere" size="0.06" rgba="0.95 0.75 0.10 1" mass="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="turret" joint="turret_j" kv="20" ctrlrange="-7 7" forcerange="-30 30"/>
  </actuator>
</mujoco>
"""
