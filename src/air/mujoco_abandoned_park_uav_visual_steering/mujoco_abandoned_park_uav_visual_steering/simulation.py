"""MuJoCo simulation boundary / MuJoCo 仿真边界。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import mujoco
import numpy as np
import yaml


@dataclass(frozen=True)
class SimulationConfig:
    """Validated runtime parameters / 已校验的运行参数。"""

    seed: int
    timestep_seconds: float
    control_hz: int
    episode_seconds: float
    render_width: int
    render_height: int
    camera_name: str
    initial_position: tuple[float, float, float]
    initial_yaw_degrees: float
    forward_speed_mps: float
    steering_rate_rps: float
    path_half_width_m: float

    @classmethod
    def from_yaml(cls, path: Path) -> "SimulationConfig":
        """Load configuration from a YAML file / 从 YAML 文件加载配置。"""
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, Mapping) or not isinstance(document.get("simulation"), Mapping):
            raise ValueError(f"Missing 'simulation' mapping / 缺少 simulation 配置段: {path}")
        raw: Mapping[str, Any] = document["simulation"]
        position = _float_tuple(raw.get("initial_position"), "initial_position", 3)
        config = cls(
            seed=_positive_int(raw.get("seed"), "seed"),
            timestep_seconds=_positive_float(raw.get("timestep_seconds"), "timestep_seconds"),
            control_hz=_positive_int(raw.get("control_hz"), "control_hz"),
            episode_seconds=_positive_float(raw.get("episode_seconds"), "episode_seconds"),
            render_width=_positive_int(raw.get("render_width"), "render_width"),
            render_height=_positive_int(raw.get("render_height"), "render_height"),
            camera_name=_non_empty_string(raw.get("camera_name"), "camera_name"),
            initial_position=position,
            initial_yaw_degrees=float(raw.get("initial_yaw_degrees", 0.0)),
            forward_speed_mps=_positive_float(raw.get("forward_speed_mps"), "forward_speed_mps"),
            steering_rate_rps=_positive_float(raw.get("steering_rate_rps"), "steering_rate_rps"),
            path_half_width_m=_positive_float(raw.get("path_half_width_m"), "path_half_width_m"),
        )
        if config.control_hz * config.timestep_seconds > 1.0:
            raise ValueError("control_hz and timestep are inconsistent / 控制频率与仿真步长不一致")
        return config


class UavSimulation:
    """Deterministic kinematic flight over a dynamic MuJoCo model / 基于 MuJoCo 模型的确定性运动学飞行。"""

    def __init__(self, model_path: Path, config: SimulationConfig) -> None:
        if not model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found / 找不到 MuJoCo 模型: {model_path}")
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "uav")
        self.camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, config.camera_name
        )
        if self.body_id < 0 or self.camera_id < 0:
            raise ValueError("Required UAV body or camera is absent / 缺少无人机机体或相机")
        self._yaw_radians = 0.0
        self._renderer: mujoco.Renderer | None = None
        self.reset()

    @property
    def control_steps(self) -> int:
        """Physics steps per control tick / 每个控制周期包含的物理步数。"""
        return max(1, round(1.0 / (self.config.control_hz * self.model.opt.timestep)))

    @property
    def position(self) -> np.ndarray:
        """Return a copy of UAV world position / 返回无人机世界坐标副本。"""
        return self.data.qpos[:3].copy()

    def reset(self, lateral_offset_m: float = 0.0) -> None:
        """Reset to the configured pose / 重置至配置姿态。"""
        if abs(lateral_offset_m) > self.config.path_half_width_m * 1.8:
            raise ValueError("lateral offset is outside the scene / 横向偏移超出场景范围")
        mujoco.mj_resetData(self.model, self.data)
        x, y, z = self.config.initial_position
        self.data.qpos[:3] = (x + lateral_offset_m, y, z)
        self._yaw_radians = np.deg2rad(self.config.initial_yaw_degrees)
        self._write_yaw_quaternion()
        mujoco.mj_forward(self.model, self.data)

    def step(self, steering: float = 0.0, forward_speed_mps: float | None = None) -> None:
        """Advance one control tick / 推进一个控制周期。"""
        if not -1.0 <= steering <= 1.0:
            raise ValueError("steering must be in [-1, 1] / steering 必须位于 [-1, 1]")
        speed = self.config.forward_speed_mps if forward_speed_mps is None else forward_speed_mps
        if speed < 0.0:
            raise ValueError("forward speed cannot be negative / 前进速度不能为负数")
        dt = self.control_steps * self.model.opt.timestep
        self._yaw_radians += float(steering) * self.config.steering_rate_rps * dt
        self.data.qpos[0] += -np.sin(self._yaw_radians) * speed * dt
        self.data.qpos[1] += np.cos(self._yaw_radians) * speed * dt
        self.data.qpos[2] = self.config.initial_position[2]
        self._write_yaw_quaternion()
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.data.time += dt

    def state(self) -> dict[str, float]:
        """Return serializable navigation state / 返回可序列化导航状态。"""
        return {
            "time_seconds": float(self.data.time),
            "x_m": float(self.data.qpos[0]),
            "y_m": float(self.data.qpos[1]),
            "z_m": float(self.data.qpos[2]),
            "yaw_radians": float(self._yaw_radians),
        }

    def capture_rgb(self) -> np.ndarray:
        """Capture the forward camera as RGB uint8 / 以 RGB uint8 捕获前视相机。"""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model,
                height=self.config.render_height,
                width=self.config.render_width,
            )
        self._renderer.update_scene(self.data, camera=self.camera_id)
        frame = self._renderer.render()
        expected_shape = (self.config.render_height, self.config.render_width, 3)
        if frame.shape != expected_shape or frame.dtype != np.uint8:
            raise RuntimeError(
                f"Unexpected RGB frame / RGB 图像规格异常: {frame.shape}, {frame.dtype}"
            )
        return frame.copy()

    def close(self) -> None:
        """Release rendering resources / 释放渲染资源。"""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def __enter__(self) -> "UavSimulation":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _write_yaw_quaternion(self) -> None:
        half = self._yaw_radians / 2.0
        self.data.qpos[3:7] = (np.cos(half), 0.0, 0.0, np.sin(half))


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer / {name} 必须为正整数")
    return value


def _positive_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"{name} must be positive / {name} 必须为正数")
    return float(value)


def _non_empty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string / {name} 必须为非空字符串")
    return value.strip()


def _float_tuple(value: Any, name: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers / {name} 必须包含 {length} 个数值")
    try:
        return tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} contains a non-number / {name} 含有非数值项") from error

