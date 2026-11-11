"""配置加载（PyYAML，ROS 2 环境自带）。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = ROOT / "config" / "simulation.yaml"
MODEL = ROOT / "models" / "arm.xml"

MUSCLES = (
    "shoulder_flexor",
    "shoulder_extensor",
    "elbow_flexor",
    "elbow_extensor",
    "biarticular_flexor",
    "biarticular_extensor",
)


@dataclass(frozen=True)
class ModelConfig:
    upper_arm: float = 0.30
    forearm: float = 0.28
    shoulder_min: float = -1.25
    shoulder_max: float = 1.45
    elbow_min: float = -0.10
    elbow_max: float = 2.30


@dataclass(frozen=True)
class SimConfig:
    timestep: float = 0.002
    control_period: float = 0.010
    steps_per_control: int = 5


@dataclass(frozen=True)
class ControlConfig:
    kp_shoulder: float = 100.0
    ki_shoulder: float = 30.0
    kd_shoulder: float = 8.0
    kp_elbow: float = 70.0
    ki_elbow: float = 20.0
    kd_elbow: float = 5.0
    biarticular_share: float = 0.5
    target_x: float = 0.38
    target_z: float = -0.17


@dataclass(frozen=True)
class MuscleConfig:
    f_max: float = 300.0
    moment_arm_shoulder: float = 0.030
    moment_arm_elbow: float = 0.025
    moment_arm_biarticular: float = 0.020


@dataclass(frozen=True)
class Config:
    model: ModelConfig
    sim: SimConfig
    ctrl: ControlConfig
    muscle: MuscleConfig


def _build(cls, raw):
    unknown = set(raw or {}) - set(cls.__dataclass_fields__)
    if unknown:
        raise ValueError(f"{cls.__name__} 含未知键：{sorted(unknown)}")
    return cls(**(raw or {}))


def load_config(path=None) -> Config:
    path = Path(path) if path is not None else DEFAULT_CONFIG
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    cfg = Config(
        model=_build(ModelConfig, raw.get("model")),
        sim=_build(SimConfig, raw.get("sim")),
        ctrl=_build(ControlConfig, raw.get("ctrl")),
        muscle=_build(MuscleConfig, raw.get("muscle")),
    )
    validate(cfg)
    return cfg


def validate(cfg: Config) -> None:
    m, s, c, mu = cfg.model, cfg.sim, cfg.ctrl, cfg.muscle
    if m.upper_arm <= 0 or m.forearm <= 0:
        raise ValueError("连杆长度必须 > 0")
    if m.shoulder_min >= m.shoulder_max or m.elbow_min >= m.elbow_max:
        raise ValueError("关节下限必须小于上限")
    if s.timestep <= 0 or s.control_period <= 0:
        raise ValueError("时间常数必须 > 0")
    if s.steps_per_control < 1:
        raise ValueError("steps_per_control 必须 >= 1")
    expected = s.timestep * s.steps_per_control
    if abs(expected - s.control_period) > 1e-9:
        raise ValueError(
            f"control_period({s.control_period}) != timestep×steps({expected})"
        )
    if c.kp_shoulder <= 0 or c.kp_elbow <= 0:
        raise ValueError("PD 比例增益必须 > 0")
    if not 0.0 <= c.biarticular_share <= 1.0:
        raise ValueError("biarticular_share 必须在 [0, 1]")
    if mu.f_max <= 0:
        raise ValueError("肌肉最大力必须 > 0")
