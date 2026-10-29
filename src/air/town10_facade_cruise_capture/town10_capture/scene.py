"""带显式 UV 的实采纹理立面与力驱动平移飞行器。"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

RAY_ANGLES = tuple((a, b) for b in (-0.18, 0.0, 0.18) for a in (-0.24, 0.0, 0.24))


def build_scene(style: str = "brick") -> mujoco.MjModel:
    if style not in {"brick", "gray", "glass"}:
        raise ValueError("style 必须为 brick、gray、glass")
    texture = Path(__file__).resolve().parent / "assets" / f"{style}.png"
    root = ET.Element("mujoco", model="town10_facade_capture")
    ET.SubElement(root, "option", timestep="0.01", gravity="0 0 -9.81")
    ET.SubElement(root, "visual").append(
        ET.Element("global", offwidth="640", offheight="480")
    )
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="facade", type="2d", file=str(texture))
    ET.SubElement(
        asset,
        "material",
        name="facade",
        texture="facade",
        texuniform="false",
        texrepeat="1 1",
        specular="0",
        shininess="0",
    )
    # 法向朝 -y，四个顶点的 UV 与墙面一一映射，避免 box 的自动重复纹理。
    ET.SubElement(
        asset,
        "mesh",
        name="panel",
        inertia="shell",
        vertex="-5 0 0  5 0 0  5 0 8  -5 0 8",
        face="0 1 2  0 2 3",
        texcoord="0 0  1 0  1 1  0 1",
    )
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world,
        "light",
        pos="0 -8 10",
        dir="0 1 -1",
        diffuse=".9 .9 .9",
        ambient=".45 .45 .45",
    )
    ET.SubElement(world, "geom", type="plane", size="20 20 .1", rgba=".21 .26 .24 1")
    ET.SubElement(
        world,
        "geom",
        name="wall_collision",
        type="box",
        pos="0 .15 4",
        size="5 .15 4",
        rgba=".4 .4 .4 1",
    )
    ET.SubElement(
        world,
        "geom",
        name="wall_texture",
        type="mesh",
        mesh="panel",
        pos="0 -.002 0",
        material="facade",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        world,
        "camera",
        name="overview",
        pos="12 -17 12",
        xyaxes=".82 .57 0 -.23 .33 .91",
    )
    body = ET.SubElement(world, "body", name="uav", pos="0 -4 1.5")
    for axis, vector in zip("xyz", ("1 0 0", "0 1 0", "0 0 1")):
        ET.SubElement(
            body, "joint", name=axis, type="slide", axis=vector, damping=".05"
        )
    ET.SubElement(
        body,
        "geom",
        name="body",
        type="box",
        size=".15 .12 .06",
        mass=".8",
        rgba=".95 .4 .08 1",
    )
    for x in (-0.24, 0.24):
        for y in (-0.24, 0.24):
            ET.SubElement(
                body,
                "geom",
                type="cylinder",
                pos=f"{x} {y} 0",
                size=".11 .015",
                mass=".05",
                rgba=".1 .12 .14 1",
            )
    ET.SubElement(
        body,
        "camera",
        name="front",
        pos="0 .13 .02",
        quat=".70710678 .70710678 0 0",
        fovy="50",
    )
    for i, (yaw, pitch) in enumerate(RAY_ANGLES):
        direction = (
            math.sin(yaw) * math.cos(pitch),
            math.cos(yaw) * math.cos(pitch),
            math.sin(pitch),
        )
        ET.SubElement(
            body,
            "site",
            name=f"ray_{i}",
            pos="0 .14 0",
            zaxis=" ".join(map(str, direction)),
            size=".005",
            rgba="1 0 0 1",
        )
    ET.SubElement(body, "site", name="imu", size=".005")
    actuators = ET.SubElement(root, "actuator")
    for axis in "xyz":
        ET.SubElement(
            actuators, "motor", joint=axis, ctrlrange="-30 30", ctrllimited="true"
        )
    sensors = ET.SubElement(root, "sensor")
    for i in range(9):
        ET.SubElement(sensors, "rangefinder", site=f"ray_{i}", cutoff="12")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def ray_features(distances: np.ndarray) -> np.ndarray:
    values = np.asarray(distances, dtype=float)
    if values.shape != (9,):
        raise ValueError("测距输入须为九个数值")
    projection = np.array([math.cos(a) * math.cos(b) for a, b in RAY_ANGLES])
    return np.where(values > 0, values * projection + 0.14, np.nan)


class MujocoBackend:
    """三平移自由度；固定姿态由内层稳定器假设提供，不直接改写积分位姿。"""

    def __init__(self, style="brick", distance=4.0):
        self.model = build_scene(style)
        self.data = mujoco.MjData(self.model)
        self.origin = np.array([0.0, -4.0, 1.5])
        self.data.qpos[:] = [-3.0, 4.0 - distance, 0.0]
        mujoco.mj_forward(self.model, self.data)
        self.mass = float(self.model.body_mass.sum())
        self.renderer = None

    def state(self):
        return (
            self.origin + self.data.qpos[:3],
            self.data.qvel[:3].copy(),
            np.array([0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]),
        )

    def ranges(self):
        return self.data.sensordata[:9].copy()

    def step(self, command, dt=0.05):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("速度指令必须为三个有限数")
        ratio = dt / self.model.opt.timestep
        if (
            not math.isfinite(dt)
            or dt <= 0
            or not math.isclose(ratio, round(ratio), abs_tol=1e-8)
        ):
            raise ValueError("控制周期必须为物理积分步长的正整数倍")
        velocity = self.data.qvel[:3].copy()
        acceleration = np.clip(4.0 * (np.asarray(command) - velocity), -8.0, 8.0)
        self.data.ctrl[:] = self.mass * (acceleration + np.array([0.0, 0.0, 9.81]))
        for _ in range(round(dt / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)

    def image(self, camera="front"):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=240, width=320)
        options = mujoco.MjvOption()
        options.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self.renderer.update_scene(self.data, camera=camera, scene_option=options)
        return self.renderer.render().copy()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
