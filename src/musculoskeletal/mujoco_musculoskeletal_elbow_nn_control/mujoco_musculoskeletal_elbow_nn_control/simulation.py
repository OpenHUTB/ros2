"""MuJoCo adapter with explicit model-name and command validation."""

from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

from .safety import ActivationLimiter


@dataclass(frozen=True)
class ElbowObservation:
    time_s: float
    angle_rad: float
    angular_velocity_rad_s: float
    flexor_force_n: float
    extensor_force_n: float


class ElbowSimulation:
    """Own the MuJoCo model and expose only task-relevant state."""

    REQUIRED_NAMES = {
        "joint": ("elbow_joint",),
        "actuator": ("flexor_muscle", "extensor_muscle"),
    }

    def __init__(self, model_path: Path, load_kg: float = 0.0) -> None:
        if load_kg < 0.0 or load_kg > 2.0:
            raise ValueError("forearm load must be within [0, 2] kg")
        try:
            import mujoco
        except ImportError as error:
            raise RuntimeError(
                "MuJoCo 3.12.0 is required to run the simulation"
            ) from error
        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
        self.data = mujoco.MjData(self.model)
        self.limiter = ActivationLimiter()
        self._validate_names()
        self._joint_qpos = int(
            self.model.jnt_qposadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, "elbow_joint")]
        )
        self._joint_qvel = int(
            self.model.jnt_dofadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, "elbow_joint")]
        )
        self._flexor_id = self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, "flexor_muscle")
        self._extensor_id = self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, "extensor_muscle")
        payload_id = self._id(mujoco.mjtObj.mjOBJ_BODY, "payload")
        self.model.body_mass[payload_id] = load_kg
        mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def _id(self, object_type, name: str) -> int:
        object_id = self._mj.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"model element is missing: {name}")
        return int(object_id)

    def _validate_names(self) -> None:
        object_types = {
            "joint": self._mj.mjtObj.mjOBJ_JOINT,
            "actuator": self._mj.mjtObj.mjOBJ_ACTUATOR,
        }
        for category, names in self.REQUIRED_NAMES.items():
            for name in names:
                self._id(object_types[category], name)

    @property
    def timestep_s(self) -> float:
        return float(self.model.opt.timestep)

    def observe(self) -> ElbowObservation:
        forces = self.data.actuator_force
        return ElbowObservation(
            time_s=float(self.data.time),
            angle_rad=float(self.data.qpos[self._joint_qpos]),
            angular_velocity_rad_s=float(self.data.qvel[self._joint_qvel]),
            flexor_force_n=abs(float(forces[self._flexor_id])),
            extensor_force_n=abs(float(forces[self._extensor_id])),
        )

    def reset(self, angle_rad: float) -> ElbowObservation:
        lower, upper = self.model.jnt_range[
            self._id(self._mj.mjtObj.mjOBJ_JOINT, "elbow_joint")
        ]
        if not float(lower) <= angle_rad <= float(upper):
            raise ValueError("initial elbow angle is outside the joint range")
        self._mj.mj_resetData(self.model, self.data)
        self.data.qpos[self._joint_qpos] = float(angle_rad)
        self._mj.mj_forward(self.model, self.data)
        return self.observe()

    def step(self, activations: Tuple[float, float]) -> ElbowObservation:
        flexor, extensor = self.limiter.apply(activations)
        self.data.ctrl[self._flexor_id] = flexor
        self.data.ctrl[self._extensor_id] = extensor
        self._mj.mj_step(self.model, self.data)
        return self.observe()

    def safe_stop(self) -> None:
        self.data.ctrl[self._flexor_id] = 0.0
        self.data.ctrl[self._extensor_id] = 0.0
