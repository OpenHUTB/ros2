"""MuJoCo 双轮差速小车与电机温度仿真。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from mujoco_motor_thermal_control.thermal import MotorThermalModel


MODEL_XML = r"""
<mujoco model="thermal_diffdrive">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.01" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual><global offwidth="720" offheight="480"/></visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.72 0.82 0.90" rgb2="0.95 0.96 0.98" width="128" height="128"/>
  </asset>
  <default>
    <joint damping="0.02" armature="0.01"/>
    <geom friction="1.2 0.01 0.001" condim="3"/>
    <motor ctrllimited="true" ctrlrange="-2.5 2.5"/>
  </default>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="12 12 0.1" rgba="0.85 0.87 0.89 1"/>
    <body name="chassis" pos="0 0 0.13">
      <freejoint name="root"/>
      <geom name="body" type="box" size="0.22 0.14 0.06" mass="5" rgba="0.05 0.48 0.39 1"/>
      <body name="left_wheel" pos="0 0.16 -0.06">
        <joint name="left_hinge" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size="0.07 0.025" mass="0.55" quat="0.7071068 0.7071068 0 0" rgba="0.15 0.17 0.19 1"/>
      </body>
      <body name="right_wheel" pos="0 -0.16 -0.06">
        <joint name="right_hinge" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size="0.07 0.025" mass="0.55" quat="0.7071068 0.7071068 0 0" rgba="0.15 0.17 0.19 1"/>
      </body>
      <body name="front_caster" pos="0.16 0 -0.105">
        <joint type="ball" damping="0"/>
        <geom type="sphere" size="0.025" mass="0.05" friction="0.05 0.01 0.001" rgba="0.85 0.45 0.12 1"/>
      </body>
      <body name="rear_caster" pos="-0.16 0 -0.105">
        <joint type="ball" damping="0"/>
        <geom type="sphere" size="0.025" mass="0.05" friction="0.05 0.01 0.001" rgba="0.85 0.45 0.12 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="left_drive" joint="left_hinge"/>
    <motor name="right_drive" joint="right_hinge"/>
  </actuator>
</mujoco>
"""


@dataclass
class SimulationState:
    time_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    pitch_rad: float
    speed_mps: float
    left_temperature_c: float
    right_temperature_c: float
    left_torque_nm: float
    right_torque_nm: float


class DifferentialDriveSimulation:
    def __init__(self, ambient_c: float = 25.0) -> None:
        self.model = mujoco.MjModel.from_xml_string(MODEL_XML)
        self.data = mujoco.MjData(self.model)
        self.ambient_c = float(ambient_c)
        self.left_motor = MotorThermalModel(temperature_c=self.ambient_c)
        self.right_motor = MotorThermalModel(temperature_c=self.ambient_c)
        self.track_m = 0.32
        self.wheel_radius_m = 0.07
        # Proportional wheel-speed response tuned for the 10 ms model step.
        self.speed_gain = 0.45
        # Torque is bounded by the actuator control range in MODEL_XML.
        self.max_wheel_speed = 24.0
        self.wheel_dof_addresses = tuple(
            int(self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)])
            for name in ("left_hinge", "right_hinge")
        )

    def step(self, linear_mps: float, angular_rps: float) -> SimulationState:
        dt = float(self.model.opt.timestep)
        left_target = (linear_mps - angular_rps * self.track_m / 2) / self.wheel_radius_m
        right_target = (linear_mps + angular_rps * self.track_m / 2) / self.wheel_radius_m
        # 24 rad/s corresponds to 1.68 m/s at the 0.07 m wheel radius.
        left_target = float(np.clip(left_target, -self.max_wheel_speed, self.max_wheel_speed))
        right_target = float(np.clip(right_target, -self.max_wheel_speed, self.max_wheel_speed))
        wheel_speed = self.data.qvel[list(self.wheel_dof_addresses)].copy()
        torque = np.clip(
            self.speed_gain * (np.array([left_target, right_target]) - wheel_speed),
            -2.5,
            2.5,
        )
        self.data.ctrl[:] = torque
        mujoco.mj_step(self.model, self.data)
        left_temp = self.left_motor.update(float(torque[0]), dt, self.ambient_c)
        right_temp = self.right_motor.update(float(torque[1]), dt, self.ambient_c)

        qpos = self.data.qpos
        qvel = self.data.qvel
        yaw = np.arctan2(
            2.0 * (qpos[3] * qpos[6] + qpos[4] * qpos[5]),
            1.0 - 2.0 * (qpos[5] ** 2 + qpos[6] ** 2),
        )
        pitch = np.arcsin(np.clip(2.0 * (qpos[3] * qpos[5] - qpos[6] * qpos[4]), -1.0, 1.0))
        return SimulationState(
            time_s=float(self.data.time),
            x_m=float(qpos[0]),
            y_m=float(qpos[1]),
            yaw_rad=float(yaw),
            pitch_rad=float(pitch),
            speed_mps=float(np.linalg.norm(qvel[:2])),
            left_temperature_c=left_temp,
            right_temperature_c=right_temp,
            left_torque_nm=float(torque[0]),
            right_torque_nm=float(torque[1]),
        )


def run_constant_command(
    steps: int,
    linear_mps: float = 0.55,
    angular_rps: float = 0.0,
    ambient_c: float = 25.0,
    *,
    render_path: str | Path | None = None,
) -> list[SimulationState]:
    if steps <= 0:
        raise ValueError("steps must be positive")
    simulation = DifferentialDriveSimulation(ambient_c=ambient_c)
    states = [simulation.step(linear_mps, angular_rps) for _ in range(steps)]
    if render_path is not None:
        from mujoco_motor_thermal_control.render import render_snapshot

        render_snapshot(simulation.model, simulation.data, render_path)
    return states
