"""MuJoCo adapter for the two-joint Hill-muscle arm."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

from .controller import MUSCLE_NAMES
from .kinematics import forward_kinematics
from .safety import ActivationLimiter


@dataclass(frozen=True)
class ArmObservation:
    time_s: float
    shoulder_rad: float
    elbow_rad: float
    shoulder_velocity_rad_s: float
    elbow_velocity_rad_s: float
    muscle_forces_n: Tuple[float, ...]
    end_effector_x_m: float
    end_effector_z_m: float


class ArmSimulation:
    """Own the MuJoCo model and expose task-level state."""

    REQUIRED_JOINTS = ('shoulder_joint', 'elbow_joint')

    def __init__(
        self,
        model_path: Path,
        payload_kg: float = 0.0,
    ) -> None:
        if payload_kg < 0.0 or payload_kg > 2.0:
            raise ValueError('payload must be within [0, 2] kg')
        try:
            import mujoco
        except ImportError as error:
            raise RuntimeError('MuJoCo is required to run the arm model') from error

        self._mj = mujoco
        xml = model_path.read_text(encoding='utf-8')
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.limiter = ActivationLimiter(len(MUSCLE_NAMES))
        self._joint_ids = tuple(
            self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.REQUIRED_JOINTS
        )
        self._actuator_ids = tuple(
            self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in MUSCLE_NAMES
        )
        payload_id = self._id(mujoco.mjtObj.mjOBJ_BODY, 'payload')
        self.model.body_mass[payload_id] = float(payload_kg) + 0.001
        mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def _id(self, object_type, name: str) -> int:
        object_id = self._mj.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError('model element is missing: %s' % name)
        return int(object_id)

    @property
    def timestep_s(self) -> float:
        return float(self.model.opt.timestep)

    def observe(self) -> ArmObservation:
        qpos = self.data.qpos
        qvel = self.data.qvel
        shoulder = float(qpos[self.model.jnt_qposadr[self._joint_ids[0]]])
        elbow = float(qpos[self.model.jnt_qposadr[self._joint_ids[1]]])
        end_x, end_z = forward_kinematics(shoulder, elbow)
        return ArmObservation(
            time_s=float(self.data.time),
            shoulder_rad=shoulder,
            elbow_rad=elbow,
            shoulder_velocity_rad_s=float(
                qvel[self.model.jnt_dofadr[self._joint_ids[0]]]
            ),
            elbow_velocity_rad_s=float(
                qvel[self.model.jnt_dofadr[self._joint_ids[1]]]
            ),
            muscle_forces_n=tuple(
                abs(float(self.data.actuator_force[index]))
                for index in self._actuator_ids
            ),
            end_effector_x_m=end_x,
            end_effector_z_m=end_z,
        )

    def reset(self, shoulder_rad: float, elbow_rad: float) -> ArmObservation:
        self._mj.mj_resetData(self.model, self.data)
        for joint_id, angle in zip(self._joint_ids, (shoulder_rad, elbow_rad)):
            lower, upper = self.model.jnt_range[joint_id]
            if not float(lower) <= angle <= float(upper):
                raise ValueError('initial joint angle is outside its range')
            self.data.qpos[self.model.jnt_qposadr[joint_id]] = float(angle)
        self._mj.mj_forward(self.model, self.data)
        return self.observe()

    def step(self, activations: Tuple[float, ...]) -> ArmObservation:
        command = self.limiter.apply(activations)
        for actuator_id, value in zip(self._actuator_ids, command):
            self.data.ctrl[actuator_id] = value
        self._mj.mj_step(self.model, self.data)
        return self.observe()

    def safe_stop(self) -> None:
        for actuator_id in self._actuator_ids:
            self.data.ctrl[actuator_id] = 0.0
