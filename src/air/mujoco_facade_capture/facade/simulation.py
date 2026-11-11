"""力驱动的幕墙定距扫描与相机采集。"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Callable

import mujoco
import numpy as np
from PIL import Image

from .route import scan_route
from .scene import build_scene, ray_features


class FacadeSimulation:
    """三个平移自由度、固定姿态的稳定飞行器简化模型。

    控制器输出世界坐标速度，内环通过力驱动关节；不会在积分中改写位姿。
    姿态稳定作为模型假设，旋翼转速及姿态控制不属于本模块。
    """

    def __init__(self, style: str = "grid", distance: float = 4., offset: float = 0.):
        self.model = build_scene(style)
        self.data = mujoco.MjData(self.model)
        self.origin = np.array([0., -4., 1.5])
        self.data.qpos[:] = [-3., 4. - distance + offset, 0.]
        mujoco.mj_forward(self.model, self.data)
        self.mass = float(self.model.body_mass.sum())
        self.renderer: mujoco.Renderer | None = None

    @property
    def position(self) -> np.ndarray:
        return self.origin + self.data.qpos[:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.data.qvel[:3].copy()

    def ranges(self) -> np.ndarray:
        return self.data.sensordata[:9].copy()

    def image(self, camera: str = "front", width: int = 320, height: int = 240) -> np.ndarray:
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        options = mujoco.MjvOption()
        options.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self.renderer.update_scene(self.data, camera=camera, scene_option=options)
        return self.renderer.render().copy()

    def step(self, commanded_velocity: np.ndarray) -> None:
        command = np.asarray(commanded_velocity, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("速度指令必须是三个有限数值")
        command = np.clip(command, -1.2, 1.2)
        acceleration = np.clip(4. * (command - self.velocity), -8., 8.)
        self.data.ctrl[:] = self.mass * (acceleration + np.array([0., 0., 9.81]))
        mujoco.mj_step(self.model, self.data)

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


def expert_velocity(error: np.ndarray, velocity: np.ndarray | None = None) -> np.ndarray:
    """输出幅值受限的比例速度；训练标签可增加速度阻尼。"""
    command = 1.1 * np.asarray(error)
    if velocity is not None:
        command -= .5 * np.asarray(velocity)
    return np.clip(command, -1., 1.)


def run_scan(output: Path, style: str = "grid", duration: float = 70.,
             distance: float = 4., offset: float = 0., capture: bool = True,
             policy: Callable[[np.ndarray, np.ndarray, np.ndarray], np.ndarray] | None = None,
             frame_hook: Callable[[FacadeSimulation, np.ndarray, np.ndarray], None] | None = None,
             control_hook: Callable[[np.ndarray, np.ndarray, np.ndarray, np.ndarray], None] | None = None) -> dict:
    """执行完整扫描；输出轨迹、照片与带单位的实测指标。"""
    if duration <= 0 or not np.isfinite(duration):
        raise ValueError("运行时长必须为正的有限数")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    sim = FacadeSimulation(style, distance, offset)
    route = scan_route(distance=distance)
    index = 0
    rows, images = [], []
    try:
        for step in range(int(duration / sim.model.opt.timestep)):
            target = route[index]
            error = target - sim.position
            projected = ray_features(sim.ranges())
            measured_distance = float(np.nanmedian(projected))
            error[1] = measured_distance - distance
            velocity = sim.velocity
            if not np.isfinite(measured_distance):
                command = np.zeros(3)
            elif policy is None:
                command = expert_velocity(error)
            else:
                command = policy(error, sim.ranges(), velocity)
            if control_hook is not None:
                control_hook(error.copy(), sim.ranges(), velocity, np.asarray(command).copy())
            sim.step(command)
            if step % 20 == 0:
                rows.append([sim.data.time, *sim.position, *sim.velocity, measured_distance, index])
                if frame_hook is not None:
                    frame_hook(sim, command, target)
            if np.linalg.norm(target - sim.position) < .12 and np.linalg.norm(sim.velocity) < .2:
                if capture:
                    filename = f"capture_{index:03d}.png"
                    Image.fromarray(sim.image()).save(output / filename)
                    images.append(filename)
                index += 1
                if index == len(route):
                    break
        if capture:
            Image.fromarray(sim.image("overview")).save(output / "overview.png")
    finally:
        sim.close()
    records = np.asarray(rows)
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "distance_m", "waypoint"])
        writer.writerows(rows)
    metrics = {
        "style": style, "controller": "neural" if policy else "proportional",
        "waypoints_total": len(route), "waypoints_completed": index,
        "completion_ratio": index / len(route), "duration_s": float(records[-1, 0]),
        "distance_rmse_m": float(np.sqrt(np.mean((records[:, 7] - distance) ** 2))),
        "minimum_wall_distance_m": float(np.min(records[:, 7])),
        "maximum_speed_m_s": float(np.max(np.linalg.norm(records[:, 4:7], axis=1))),
        "images": images,
    }
    (output / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    return metrics
