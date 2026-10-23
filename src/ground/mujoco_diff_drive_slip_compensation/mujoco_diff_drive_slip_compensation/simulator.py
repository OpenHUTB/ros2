"""Small MuJoCo adapter with explicit model-name validation."""

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
    WHEEL_JOINT_NAMES = (
        'left_front_joint',
        'left_rear_joint',
        'right_front_joint',
        'right_rear_joint',
    )

    def __init__(self, model_path: Path) -> None:
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError(
                '缺少 mujoco 3.12.0 / mujoco 3.12.0 is required'
            ) from exc

        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self._actuator_ids = tuple(
            self._require_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.ACTUATOR_NAMES
        )
        self._chassis_body_id = self._require_id(
            mujoco.mjtObj.mjOBJ_BODY, 'chassis'
        )
        self._ground_geom_id = self._require_id(
            mujoco.mjtObj.mjOBJ_GEOM, 'ground'
        )
        self._wheel_joint_ids = tuple(
            self._require_id(mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.WHEEL_JOINT_NAMES
        )
        self._accelerometer_sensor_id = self._require_id(
            mujoco.mjtObj.mjOBJ_SENSOR, 'imu_accelerometer'
        )
        self._gyroscope_sensor_id = self._require_id(
            mujoco.mjtObj.mjOBJ_SENSOR, 'imu_gyroscope'
        )
        self._initial_qpos = self.data.qpos.copy()
        self._initial_qvel = self.data.qvel.copy()

    def _require_id(self, object_type: Any, name: str) -> int:
        object_id = int(self._mujoco.mj_name2id(self.model, object_type, name))
        if object_id < 0:
            raise ValueError(f'MJCF object not found: {name}')
        return object_id

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def apply_wheel_speeds(self, left: float, right: float) -> None:
        commands = (left, left, right, right)
        for actuator_id, command in zip(self._actuator_ids, commands):
            lower, upper = self.model.actuator_ctrlrange[actuator_id]
            self.data.ctrl[actuator_id] = float(np.clip(command, lower, upper))


    def set_ground_friction(self, coefficient: float) -> None:
        """Update sliding friction while preserving torsional/rolling values."""
        if not np.isfinite(coefficient) or coefficient <= 0.0:
            raise ValueError('friction coefficient must be finite and positive')
        self.model.geom_friction[self._ground_geom_id, 0] = coefficient

    def wheel_state(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Return wheel joint positions and velocities in canonical order."""
        positions = tuple(
            float(self.data.qpos[self.model.jnt_qposadr[joint_id]])
            for joint_id in self._wheel_joint_ids
        )
        velocities = tuple(
            float(self.data.qvel[self.model.jnt_dofadr[joint_id]])
            for joint_id in self._wheel_joint_ids
        )
        return positions, velocities

    def _sensor_vector(self, sensor_id: int) -> Tuple[float, float, float]:
        address = int(self.model.sensor_adr[sensor_id])
        dimension = int(self.model.sensor_dim[sensor_id])
        if dimension != 3:
            raise RuntimeError('expected a three-axis MuJoCo sensor')
        values = self.data.sensordata[address:address + dimension]
        return tuple(float(value) for value in values)

    def imu_state(
        self,
    ) -> tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """Return accelerometer and gyroscope vectors in the IMU site frame."""
        return (
            self._sensor_vector(self._accelerometer_sensor_id),
            self._sensor_vector(self._gyroscope_sensor_id),
        )

    def step(self) -> None:
        self._mujoco.mj_step(self.model, self.data)

    def reset(self, preserve_time: bool = False) -> None:
        current_time = float(self.data.time)

    def step(self) -> None:
        self._mujoco.mj_step(self.model, self.data)

    def reset(self) -> None:

        self._mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self._initial_qpos
        self.data.qvel[:] = self._initial_qvel
        self.data.ctrl[:] = 0.0
        if preserve_time:
            self.data.time = current_time
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
