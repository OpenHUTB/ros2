"""MuJoCo scene loading and camera placement / MuJoCo 场景加载与相机定位。"""

from __future__ import annotations

from pathlib import Path
from typing import Any


SCENARIOS = ("sparse", "dense", "occluded")
MODULE_ROOT = Path(__file__).resolve().parent.parent
SCENE_ROOT = MODULE_ROOT / "assets" / "mjcf"


def scenario_path(name: str) -> Path:
    """Resolve a checked-in scenario name / 解析已纳入版本控制的场景名称。"""
    if name not in SCENARIOS:
        choices = ", ".join(SCENARIOS)
        raise ValueError(f"unknown scenario {name!r}; choose one of {choices} / 未知场景")
    path = SCENE_ROOT / f"rooftop_{name}.xml"
    if not path.is_file():
        raise FileNotFoundError(f"scenario file is missing / 场景文件缺失: {path}")
    return path


def load_model(name: str) -> tuple[Any, Any]:
    """Load one MuJoCo model lazily / 延迟加载一个 MuJoCo 模型。"""
    try:
        import mujoco
    except ImportError as exc:
        raise RuntimeError(
            "MuJoCo is not installed; install requirements.txt / 未安装 MuJoCo，请安装 requirements.txt"
        ) from exc
    model = mujoco.MjModel.from_xml_path(str(scenario_path(name)))
    return model, mujoco.MjData(model)


def set_drone_pose(model: Any, data: Any, x: float, y: float, altitude: float, yaw: float = 0.0) -> None:
    """Set the free body pose while keeping the camera nadir-facing / 设置自由体位姿并保持相机朝下。"""
    import math
    import mujoco

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "uav_free_joint")
    if joint_id < 0:
        raise RuntimeError("uav_free_joint is absent / 场景缺少 uav_free_joint")
    qpos_address = model.jnt_qposadr[joint_id]
    data.qpos[qpos_address : qpos_address + 7] = (
        x,
        y,
        altitude,
        math.cos(yaw / 2.0),
        0.0,
        0.0,
        math.sin(yaw / 2.0),
    )
    mujoco.mj_forward(model, data)
