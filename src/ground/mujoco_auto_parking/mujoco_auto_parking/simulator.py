"""MuJoCo adapter for the differential-drive auto-parking car."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Tuple

import numpy as np


@dataclass(frozen=True)
class SimulationState:
    """Planar ground-truth state extracted from the MuJoCo free joint."""

    time: float
    x: float
    y: float
    yaw: float
    linear_velocity: float
    angular_velocity: float


def quaternion_to_yaw(quaternion: np.ndarray) -> float:
    """Convert a MuJoCo wxyz quaternion to planar yaw."""
    w, x, y, z = (float(value) for value in quaternion)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(sin_yaw, cos_yaw))


class MujocoSimulation:
    """Own the MuJoCo model, data and four wheel actuators."""

    ACTUATOR_NAMES = ('left_front', 'left_rear', 'right_front', 'right_rear')

    def __init__(self, model_path: Path | str) -> None:
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError(
                '缺少 mujoco 3.12.0 / mujoco 3.12.0 is required'
            ) from exc

        self._mujoco = mujoco
        self.model_path = Path(model_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self._actuator_ids = tuple(
            self._require_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.ACTUATOR_NAMES
        )
        self._chassis_body_id = self._require_id(
            mujoco.mjtObj.mjOBJ_BODY, 'chassis'
        )
        self._freejoint_id = self._require_id(
            mujoco.mjtObj.mjOBJ_JOINT, 'chassis_freejoint'
        )
        self._qpos_adr = int(self.model.jnt_qposadr[self._freejoint_id])
        self._initial_qpos = self.data.qpos.copy()
        self._initial_qvel = self.data.qvel.copy()
        self._mujoco.mj_forward(self.model, self.data)

    def _require_id(self, object_type: Any, name: str) -> int:
        object_id = int(self._mujoco.mj_name2id(self.model, object_type, name))
        if object_id < 0:
            raise ValueError(f'MJCF object not found: {name}')
        return object_id

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def object_names(self, object_type: Any) -> list[str]:
        """Return all named objects of one MuJoCo object type."""
        counts = {
            self._mujoco.mjtObj.mjOBJ_BODY: int(self.model.nbody),
            self._mujoco.mjtObj.mjOBJ_JOINT: int(self.model.njnt),
            self._mujoco.mjtObj.mjOBJ_ACTUATOR: int(self.model.nu),
            self._mujoco.mjtObj.mjOBJ_CAMERA: int(self.model.ncam),
            self._mujoco.mjtObj.mjOBJ_SITE: int(self.model.nsite),
            self._mujoco.mjtObj.mjOBJ_GEOM: int(self.model.ngeom),
        }
        names = []
        for object_id in range(counts.get(object_type, 0)):
            name = self._mujoco.mj_id2name(self.model, object_type, object_id)
            if name:
                names.append(name)
        return names

    def has_object(self, object_type: Any, name: str) -> bool:
        return int(self._mujoco.mj_name2id(self.model, object_type, name)) >= 0

    def apply_wheel_speeds(self, left: float, right: float) -> None:
        """Apply left/right wheel target speeds in rad/s."""
        commands = (left, left, right, right)
        for actuator_id, command in zip(self._actuator_ids, commands):
            value = float(command)
            if bool(self.model.actuator_ctrllimited[actuator_id]):
                lower, upper = self.model.actuator_ctrlrange[actuator_id]
                value = float(np.clip(value, lower, upper))
            self.data.ctrl[actuator_id] = value

    def step(self) -> None:
        self._mujoco.mj_step(self.model, self.data)

    def reset(self) -> None:
        self._mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self._initial_qpos
        self.data.qvel[:] = self._initial_qvel
        self.data.ctrl[:] = 0.0
        self._mujoco.mj_forward(self.model, self.data)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """Teleport the chassis to a planar pose (used by tests and training)."""
        address = self._qpos_adr
        half_yaw = 0.5 * float(yaw)
        self.data.qpos[address + 0] = float(x)
        self.data.qpos[address + 1] = float(y)
        self.data.qpos[address + 2] = float(self._initial_qpos[address + 2])
        self.data.qpos[address + 3] = float(np.cos(half_yaw))
        self.data.qpos[address + 4] = 0.0
        self.data.qpos[address + 5] = 0.0
        self.data.qpos[address + 6] = float(np.sin(half_yaw))
        self.data.qvel[:] = 0.0
        self._mujoco.mj_forward(self.model, self.data)

    def planar_state(self) -> SimulationState:
        position = self.data.xpos[self._chassis_body_id]
        quaternion = self.data.xquat[self._chassis_body_id]
        velocity = np.zeros(6, dtype=float)
        self._mujoco.mj_objectVelocity(
            self.model,
            self.data,
            self._mujoco.mjtObj.mjOBJ_BODY,
            self._chassis_body_id,
            velocity,
            flg_local=1,
        )
        return SimulationState(
            time=float(self.data.time),
            x=float(position[0]),
            y=float(position[1]),
            yaw=quaternion_to_yaw(quaternion),
            linear_velocity=float(velocity[3]),
            angular_velocity=float(velocity[2]),
        )

    def chassis_quaternion(self) -> Tuple[float, float, float, float]:
        values = self.data.xquat[self._chassis_body_id]
        return tuple(float(value) for value in values)
