"""由几何体构造立面与姿态稳定的三平移自由度无人机。"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


RAY_ANGLES = tuple((a, b) for b in (-0.18, 0.0, 0.18) for a in (-0.24, 0.0, 0.24))


def build_scene(style: str = "grid") -> mujoco.MjModel:
    """构造场景；三种立面共享物理尺度，颜色和装饰各不相同。"""
    if style not in {"grid", "brick", "plain"}:
        raise ValueError("立面类型必须为 grid、brick 或 plain")
    root = ET.Element("mujoco", model="facade_capture")
    ET.SubElement(root, "option", timestep="0.01", gravity="0 0 -9.81")
    ET.SubElement(root, "visual")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="floor", type="2d", builtin="checker", rgb1=".17 .22 .18", rgb2=".21 .28 .21", width="256", height="256")
    ET.SubElement(asset, "material", name="floor", texture="floor", texrepeat="8 8")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -6 10", dir="0 0.5 -1", diffuse=".9 .9 .9")
    ET.SubElement(world, "geom", type="plane", size="20 20 .1", material="floor")
    color = {"grid": ".52 .64 .72 1", "brick": ".65 .42 .29 1", "plain": ".7 .72 .68 1"}[style]
    ET.SubElement(world, "geom", name="facade", type="box", pos="0 .15 4", size="5 .15 4", rgba=color)
    # 装饰线条薄且不参与碰撞，不改变测距平面的几何定义。
    for x in range(-4, 5, 2):
        ET.SubElement(world, "geom", type="box", pos=f"{x} -.006 4", size=".025 .008 3.95", rgba=".15 .19 .2 1", contype="0", conaffinity="0")
    for z in range(1, 8):
        ET.SubElement(world, "geom", type="box", pos=f"0 -.006 {z}", size="4.95 .008 .02", rgba=".15 .19 .2 1", contype="0", conaffinity="0")
    ET.SubElement(world, "camera", name="overview", pos="12 -17 12", xyaxes=".82 .57 0 -.23 .33 .91")
    body = ET.SubElement(world, "body", name="uav", pos="0 -4 1.5")
    for axis, vector in zip("xyz", ("1 0 0", "0 1 0", "0 0 1")):
        ET.SubElement(body, "joint", name=axis, type="slide", axis=vector, damping=".05")
    ET.SubElement(body, "geom", name="body", type="box", size=".15 .12 .06", mass=".8", rgba=".9 .45 .08 1")
    for x in (-.24, .24):
        for y in (-.24, .24):
            ET.SubElement(body, "geom", type="cylinder", pos=f"{x} {y} 0", size=".11 .015", mass=".05", rgba=".1 .12 .14 1")
    ET.SubElement(body, "camera", name="front", pos="0 .13 .02", quat=".70710678 .70710678 0 0", fovy="50")
    for i, (yaw, pitch) in enumerate(RAY_ANGLES):
        direction = (math.sin(yaw) * math.cos(pitch), math.cos(yaw) * math.cos(pitch), math.sin(pitch))
        ET.SubElement(body, "site", name=f"ray_{i}", pos="0 .14 0", zaxis=" ".join(map(str, direction)), size=".005", rgba="1 0 0 1")
    ET.SubElement(body, "site", name="imu", size=".005")
    actuator = ET.SubElement(root, "actuator")
    for axis in "xyz":
        ET.SubElement(actuator, "motor", name=f"force_{axis}", joint=axis, ctrlrange="-30 30", ctrllimited="true")
    sensors = ET.SubElement(root, "sensor")
    for i in range(9):
        ET.SubElement(sensors, "rangefinder", name=f"distance_{i}", site=f"ray_{i}", cutoff="12")
    ET.SubElement(sensors, "velocimeter", name="velocity", site="imu")
    ET.SubElement(sensors, "accelerometer", name="acceleration", site="imu")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def ray_features(distances: np.ndarray) -> np.ndarray:
    """把九束斜向距离投影为垂直墙面距离；无回波保留为缺失值。"""
    distances = np.asarray(distances, dtype=float)
    if distances.shape != (9,):
        raise ValueError("测距输入必须包含九个数值")
    projection = np.array([math.cos(a) * math.cos(b) for a, b in RAY_ANGLES])
    return np.where(distances > 0, distances * projection + .14, np.nan)
