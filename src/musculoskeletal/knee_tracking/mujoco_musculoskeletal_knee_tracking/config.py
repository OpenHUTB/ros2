"""配置加载与校验（只用 PyYAML，ROS 2 环境自带）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

PKG_NAME = "mujoco_musculoskeletal_knee_tracking"
ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = ROOT / "config" / "params.yaml"
ASSET = ROOT / "assets" / "knee.xml"


@dataclass(frozen=True)
class SimConfig:
    dt: float = 0.005
    duration: float = 30.0
    exit_after_duration: bool = False


@dataclass(frozen=True)
class TrajectoryConfig:
    type: str = "sine"
    amplitude_deg: float = 40.0
    frequency_hz: float = 0.4
    bias_deg: float = 30.0
    step_start_deg: float = 20.0
    step_target_deg: float = 50.0
    step_time: float = 0.5


@dataclass(frozen=True)
class ControlConfig:
    kp: float = 30.0
    kd: float = 3.0
    max_torque: float = 17.5
    limit_deg: float = 90.0
    limit_low_deg: float = -15.0


@dataclass(frozen=True)
class MuscleConfig:
    f_max: float = 500.0
    moment_arm: float = 0.035
    tau_act: float = 0.02
    tau_deact: float = 0.05


@dataclass(frozen=True)
class MetricsConfig:
    band_percent: float = 2.0
    wide_band_deg: float = 5.0
    steady_window_s: float = 1.0


@dataclass(frozen=True)
class Config:
    sim: SimConfig
    trajectory: TrajectoryConfig
    ctrl: ControlConfig
    muscle: MuscleConfig
    metrics: MetricsConfig


def _build(cls, raw):
    known = set(cls.__dataclass_fields__)
    unknown = set(raw or {}) - known
    if unknown:
        raise ValueError(f"{cls.__name__} 含未知键：{sorted(unknown)}")
    return cls(**(raw or {}))


def load_config(path: str | Path | None = None) -> Config:
    path = Path(path) if path is not None else DEFAULT_CONFIG
    if not path.is_file():
        raise FileNotFoundError(f"配置文件不存在：{path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    cfg = Config(
        sim=_build(SimConfig, raw.get("sim")),
        trajectory=_build(TrajectoryConfig, raw.get("trajectory")),
        ctrl=_build(ControlConfig, raw.get("ctrl")),
        muscle=_build(MuscleConfig, raw.get("muscle")),
        metrics=_build(MetricsConfig, raw.get("metrics")),
    )
    validate(cfg)
    return cfg


def validate(cfg: Config) -> None:
    if cfg.sim.dt <= 0:
        raise ValueError("sim.dt 必须 > 0")
    if cfg.sim.duration <= 0:
        raise ValueError("sim.duration 必须 > 0")
    if cfg.trajectory.type not in ("sine", "step"):
        raise ValueError("trajectory.type 只能是 sine 或 step")
    if cfg.trajectory.frequency_hz <= 0:
        raise ValueError("frequency_hz 必须 > 0")
    lo = cfg.trajectory.bias_deg - cfg.trajectory.amplitude_deg
    hi = cfg.trajectory.bias_deg + cfg.trajectory.amplitude_deg
    if cfg.trajectory.type == "sine" and (lo < cfg.ctrl.limit_low_deg
                                          or hi > cfg.ctrl.limit_deg):
        raise ValueError(f"正弦范围 [{lo}, {hi}] 超出关节限位")
    if cfg.ctrl.kp < 0 or cfg.ctrl.kd < 0:
        raise ValueError("PD 增益必须 >= 0")
    if cfg.ctrl.max_torque <= 0:
        raise ValueError("max_torque 必须 > 0")
    if cfg.ctrl.limit_low_deg >= cfg.ctrl.limit_deg:
        raise ValueError("关节下限必须小于上限")
    if cfg.muscle.f_max <= 0 or cfg.muscle.moment_arm <= 0:
        raise ValueError("肌肉最大力与力臂必须 > 0")
    if cfg.muscle.tau_act <= 0 or cfg.muscle.tau_deact <= 0:
        raise ValueError("激活/失活时间常数必须 > 0")
    torque_limit = cfg.muscle.f_max * cfg.muscle.moment_arm
    if not math.isclose(torque_limit, cfg.ctrl.max_torque, rel_tol=0.2):
        raise ValueError(
            f"max_torque({cfg.ctrl.max_torque}) 与 f_max×moment_arm({torque_limit}) 相差过大"
        )
