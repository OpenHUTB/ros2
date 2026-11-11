"""参数配置：最小 YAML 子集解析 + 强类型 dataclass 校验。

没有引入 PyYAML，只实现本项目用得到的 YAML 子集：
- 缩进嵌套映射（section: 下一层 key: value）
- 标量：整数、浮点、布尔、null、字符串（可带引号）
- 行内列表：[a, b, c]
- 行注释：以 # 开始（引号内的 # 不算注释）

这样可以做到零第三方依赖，同时把全部参数集中在 config/params.yaml。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields, is_dataclass
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    """配置文件格式或数值非法。"""


# ---------------------------------------------------------------------------
# 最小 YAML 解析器
# ---------------------------------------------------------------------------

def _strip_comment(line: str) -> str:
    """去掉引号外的 # 注释。"""
    quote: str | None = None
    for i, ch in enumerate(line):
        if ch in ("'", '"'):
            if quote is None:
                quote = ch
            elif quote == ch:
                quote = None
        elif ch == "#" and quote is None:
            return line[:i]
    return line


def _parse_scalar(text: str) -> Any:
    text = text.strip()
    if text == "":
        raise ConfigError("冒号后缺少值")
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
        return text[1:-1]
    low = text.lower()
    if low in ("true", "yes", "on"):
        return True
    if low in ("false", "no", "off"):
        return False
    if low in ("null", "none", "~"):
        return None
    if low in {
        "nan", "+nan", "-nan", ".nan",
        "inf", "+inf", "-inf", ".inf",
        "infinity", "+infinity", "-infinity",
    }:
        raise ConfigError(f"数值必须是有限值: {text!r}")
    if text.startswith("[") and text.endswith("]"):
        body = text[1:-1].strip()
        if body == "":
            return []
        return [_parse_scalar(part) for part in body.split(",")]
    if text.lstrip("+-").isdigit():
        return int(text)
    try:
        value = float(text)
    except ValueError:
        return text
    if not math.isfinite(value):
        raise ConfigError(f"数值必须是有限值: {text!r}")
    return value


def parse_yaml(text: str) -> dict[str, Any]:
    """解析嵌套映射形式的 YAML 子集。"""
    root: dict[str, Any] = {}
    # 栈元素：(该映射的缩进, 映射对象)
    stack: list[tuple[int, dict[str, Any]]] = [(-1, root)]

    for lineno, raw in enumerate(text.splitlines(), start=1):
        if "\t" in raw:
            raise ConfigError(f"第 {lineno} 行含制表符，请使用空格缩进")
        stripped = _strip_comment(raw).rstrip()
        if not stripped.strip():
            continue
        indent = len(stripped) - len(stripped.lstrip(" "))
        content = stripped.strip()
        if ":" not in content:
            raise ConfigError(f"第 {lineno} 行缺少冒号: {content!r}")

        while len(stack) > 1 and indent <= stack[-1][0]:
            stack.pop()
        if indent <= stack[-1][0]:
            raise ConfigError(f"第 {lineno} 行缩进不一致")
        parent = stack[-1][1]

        key, _, rest = content.partition(":")
        key = key.strip().strip("'\"")
        if not key:
            raise ConfigError(f"第 {lineno} 行键名为空")
        if key in parent:
            raise ConfigError(f"第 {lineno} 行重复键: {key!r}")
        rest = rest.strip()
        if rest == "":
            node: dict[str, Any] = {}
            parent[key] = node
            stack.append((indent, node))
        else:
            parent[key] = _parse_scalar(rest)

    return root


# ---------------------------------------------------------------------------
# 参数 dataclass
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.005
    sine_duration: float = 30.0
    step_duration: float = 6.0


@dataclass(frozen=True)
class JointConfig:
    inertia: float = 0.06
    damping: float = 0.5
    gravity_torque: float = 4.5
    angle_min_deg: float = -15.0
    angle_max_deg: float = 90.0


@dataclass(frozen=True)
class MuscleConfig:
    max_force: float = 500.0
    moment_arm: float = 0.035
    tau_act: float = 0.02
    tau_deact: float = 0.05
    force_length_strength: float = 0.0
    optimal_angle_deg: float = 30.0


@dataclass(frozen=True)
class ControllerConfig:
    kp: float = 30.0
    kd: float = 3.0
    limit_epsilon_deg: float = 0.01
    gravity_feedforward: float = 1.0   # 重力矩前馈系数，1.0 为完全补偿


@dataclass(frozen=True)
class TrajectoryConfig:
    sine_bias_deg: float = 30.0
    sine_amplitude_deg: float = 40.0
    sine_frequency_hz: float = 0.4
    step_start_deg: float = 20.0
    step_target_deg: float = 50.0
    step_time: float = 0.5


@dataclass(frozen=True)
class MetricsConfig:
    step_band_percent: float = 2.0
    wide_band_deg: float = 5.0
    steady_state_window_s: float = 1.0


@dataclass(frozen=True)
class RenderConfig:
    gif_enabled: bool = True
    gif_fps: int = 15
    gif_dpi: int = 80
    gif_max_frames: int = 120


@dataclass(frozen=True)
class AppConfig:
    simulation: SimulationConfig
    joint: JointConfig
    muscle: MuscleConfig
    controller: ControllerConfig
    trajectory: TrajectoryConfig
    metrics: MetricsConfig
    render: RenderConfig


_SECTIONS: dict[str, type] = {
    "simulation": SimulationConfig,
    "joint": JointConfig,
    "muscle": MuscleConfig,
    "controller": ControllerConfig,
    "trajectory": TrajectoryConfig,
    "metrics": MetricsConfig,
    "render": RenderConfig,
}


def _build_section(name: str, cls: type, raw: dict[str, Any]) -> Any:
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ConfigError(f"配置段 {name!r} 必须是映射")
    expected = {f.name for f in fields(cls)}
    unknown = set(raw) - expected
    if unknown:
        raise ConfigError(f"配置段 {name!r} 含未知键: {sorted(unknown)}")
    try:
        return cls(**raw)
    except TypeError as exc:  # 理论上不会发生，保留可读错误
        raise ConfigError(f"配置段 {name!r} 构造失败: {exc}") from exc


def _require_finite(name: str, value: float) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} 必须是数值，实际为 {value!r}")
    if not math.isfinite(float(value)):
        raise ConfigError(f"{name} 必须是有限数值")


def validate_config(cfg: AppConfig) -> None:
    """检查物理与数值参数是否自洽；不合法时抛 ConfigError。"""
    sim, joint, muscle = cfg.simulation, cfg.joint, cfg.muscle
    ctrl, traj, met, ren = cfg.controller, cfg.trajectory, cfg.metrics, cfg.render

    for name, value in (
        ("simulation.dt", sim.dt),
        ("simulation.sine_duration", sim.sine_duration),
        ("simulation.step_duration", sim.step_duration),
        ("joint.inertia", joint.inertia),
        ("joint.damping", joint.damping),
        ("joint.gravity_torque", joint.gravity_torque),
        ("joint.angle_min_deg", joint.angle_min_deg),
        ("joint.angle_max_deg", joint.angle_max_deg),
        ("muscle.max_force", muscle.max_force),
        ("muscle.moment_arm", muscle.moment_arm),
        ("muscle.tau_act", muscle.tau_act),
        ("muscle.tau_deact", muscle.tau_deact),
        ("muscle.force_length_strength", muscle.force_length_strength),
        ("muscle.optimal_angle_deg", muscle.optimal_angle_deg),
        ("controller.kp", ctrl.kp),
        ("controller.kd", ctrl.kd),
        ("controller.limit_epsilon_deg", ctrl.limit_epsilon_deg),
        ("controller.gravity_feedforward", ctrl.gravity_feedforward),
        ("trajectory.sine_bias_deg", traj.sine_bias_deg),
        ("trajectory.sine_amplitude_deg", traj.sine_amplitude_deg),
        ("trajectory.sine_frequency_hz", traj.sine_frequency_hz),
        ("trajectory.step_start_deg", traj.step_start_deg),
        ("trajectory.step_target_deg", traj.step_target_deg),
        ("trajectory.step_time", traj.step_time),
        ("metrics.step_band_percent", met.step_band_percent),
        ("metrics.wide_band_deg", met.wide_band_deg),
        ("metrics.steady_state_window_s", met.steady_state_window_s),
    ):
        _require_finite(name, value)

    if sim.dt <= 0:
        raise ConfigError("simulation.dt 必须 > 0")
    if sim.sine_duration <= 0 or sim.step_duration <= 0:
        raise ConfigError("仿真时长必须 > 0")
    if joint.inertia <= 0:
        raise ConfigError("joint.inertia 必须 > 0")
    if joint.damping < 0:
        raise ConfigError("joint.damping 必须 >= 0")
    if joint.gravity_torque < 0:
        raise ConfigError("joint.gravity_torque 必须 >= 0")
    if joint.angle_min_deg >= joint.angle_max_deg:
        raise ConfigError("关节下限必须小于上限")
    if muscle.max_force <= 0 or muscle.moment_arm <= 0:
        raise ConfigError("肌肉最大力与力臂必须 > 0")
    if muscle.tau_act <= 0 or muscle.tau_deact <= 0:
        raise ConfigError("肌肉激活/失活时间常数必须 > 0")
    if not 0.0 <= muscle.force_length_strength <= 1.0:
        raise ConfigError("muscle.force_length_strength 必须在 [0, 1]")
    if ctrl.kp < 0 or ctrl.kd < 0:
        raise ConfigError("PD 增益必须 >= 0")
    if ctrl.limit_epsilon_deg < 0:
        raise ConfigError("limit_epsilon_deg 必须 >= 0")
    if not 0.0 <= ctrl.gravity_feedforward <= 2.0:
        raise ConfigError("gravity_feedforward 必须在 [0, 2]")
    if traj.sine_frequency_hz <= 0:
        raise ConfigError("正弦频率必须 > 0")
    if traj.sine_amplitude_deg < 0:
        raise ConfigError("正弦幅值必须 >= 0")
    if traj.step_time < 0:
        raise ConfigError("step_time 必须 >= 0")
    lo = traj.sine_bias_deg - traj.sine_amplitude_deg
    hi = traj.sine_bias_deg + traj.sine_amplitude_deg
    if lo < joint.angle_min_deg or hi > joint.angle_max_deg:
        raise ConfigError(
            f"正弦参考范围 [{lo}, {hi}] deg 超出关节限位 "
            f"[{joint.angle_min_deg}, {joint.angle_max_deg}] deg"
        )
    for name, value in (
        ("trajectory.step_start_deg", traj.step_start_deg),
        ("trajectory.step_target_deg", traj.step_target_deg),
    ):
        if not joint.angle_min_deg <= value <= joint.angle_max_deg:
            raise ConfigError(f"{name} 超出关节限位")
    if met.step_band_percent <= 0 or met.step_band_percent >= 100:
        raise ConfigError("metrics.step_band_percent 必须在 (0, 100)")
    if met.wide_band_deg <= 0:
        raise ConfigError("metrics.wide_band_deg 必须 > 0")
    if met.steady_state_window_s <= 0:
        raise ConfigError("metrics.steady_state_window_s 必须 > 0")
    if not isinstance(ren.gif_enabled, bool):
        raise ConfigError("render.gif_enabled 必须是布尔值")
    if not isinstance(ren.gif_fps, int) or not 1 <= ren.gif_fps <= 60:
        raise ConfigError("render.gif_fps 必须是 1..60 的整数")
    if not isinstance(ren.gif_max_frames, int) or ren.gif_max_frames < 1:
        raise ConfigError("render.gif_max_frames 必须是正整数")
    if not isinstance(ren.gif_dpi, int) or ren.gif_dpi < 30:
        raise ConfigError("render.gif_dpi 必须是不小于 30 的整数")


def load_config(path: str | Path) -> AppConfig:
    path = Path(path)
    if not path.is_file():
        raise ConfigError(f"配置文件不存在: {path}")
    raw = parse_yaml(path.read_text(encoding="utf-8"))
    unknown = set(raw) - set(_SECTIONS)
    if unknown:
        raise ConfigError(f"未知配置段: {sorted(unknown)}")
    sections = {
        name: _build_section(name, cls, raw.get(name, {}))
        for name, cls in _SECTIONS.items()
    }
    cfg = AppConfig(**sections)
    validate_config(cfg)
    return cfg


def to_dict(cfg: AppConfig) -> dict[str, Any]:
    """把配置转成可打印的嵌套字典（供 metrics.txt 使用）。"""
    if not is_dataclass(cfg):
        raise TypeError("cfg 必须是 dataclass")
    result: dict[str, Any] = {}
    for section in fields(cfg):
        value = getattr(cfg, section.name)
        result[section.name] = {
            f.name: getattr(value, f.name) for f in fields(value)
        }
    return result
