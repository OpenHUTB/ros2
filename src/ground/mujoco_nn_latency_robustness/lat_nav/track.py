"""赛道生成：三条中心线（圆、S 弯、8 字）→ 解析边界 → MuJoCo 场景 XML。

场景完全由代码生成（`from_xml_string`），不依赖任何资源文件路径，因此 `ros2 launch`
起来以后不会出现"把 models/ 装到 share/ 下就找不到"的问题。
"""
from __future__ import annotations

import numpy as np

SHAPES = ("circle", "kidney", "peanut")

LANE_HALF_WIDTH = 0.75      # 车道半宽（m）
WALL_SEG = 0.14             # 墙体用 14 cm 的小方块拼，接缝小于车身半径

# 三条赛道都必须**不自交**：车道半宽 0.75 m，因此最小曲率半径要大一圈，
# 否则内侧墙会在弯心处自己压到自己身上，车根本开不过去（peanut 的 k=1.0 版就是 1.08 m）。
SHAPES_SPEC = {
    "circle": "3.00 m 定半径，最小曲率半径 3.00 m",
    "kidney": "r=3+0.9·sinθ 的肾形，最小曲率半径 2.86 m",
    "peanut": "r=1.6·(3+1.2·cos2θ) 的花生形，最小曲率半径 1.73 m",
}


def centerline(shape: str, n: int = 720) -> np.ndarray:
    """返回 (n,2) 的闭合中心线，逆时针为正。"""
    t = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    if shape == "circle":
        pts = np.stack([3.0 * np.cos(t), 3.0 * np.sin(t)], axis=1)
    elif shape == "kidney":
        r = 3.0 + 0.9 * np.sin(t)
        pts = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
    elif shape == "peanut":
        r = 1.6 * (3.0 + 1.2 * np.cos(2.0 * t))
        pts = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
    else:
        raise ValueError(f"未知赛道 {shape}")
    return pts.astype(np.float64)


def tangents(path: np.ndarray) -> np.ndarray:
    """闭合曲线的单位切向。"""
    d = np.gradient(path, axis=0)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def normals(path: np.ndarray) -> np.ndarray:
    """左法向（逆时针方向为正）。"""
    t = tangents(path)
    return np.stack([-t[:, 1], t[:, 0]], axis=1)


def boundary_points(path: np.ndarray, offset: float, seg: float = WALL_SEG) -> np.ndarray:
    """按弧长把一侧边界重采样成等距点，用来摆墙块。"""
    side = path + offset * normals(path)
    closed = np.vstack([side, side[:1]])
    seg_len = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = s[-1]
    m = max(8, int(round(total / seg)))
    q = np.linspace(0.0, total, m, endpoint=False)
    xp = np.interp(q, s, closed[:, 0])
    yp = np.interp(q, s, closed[:, 1])
    return np.stack([xp, yp], axis=1)


def build_xml(shape: str, lane_half_width: float = LANE_HALF_WIDTH) -> str:
    """生成整个场景的 MJCF 字符串。"""
    path = centerline(shape)

    walls = []
    for off in (lane_half_width, -lane_half_width):
        for x, y in boundary_points(path, off):
            walls.append(
                f'<geom type="box" pos="{x:.3f} {y:.3f} 0.11" size="0.075 0.075 0.11" '
                f'rgba="0.85 0.62 0.20 1"/>')
    wall_xml = "\n    ".join(walls)

    return f"""<mujoco model="latency_track_{shape}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.005" integrator="RK4" gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="640" offheight="480"/>
    <map znear="0.02" zfar="40"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.16 0.20 0.26"
             rgb2="0.22 0.27 0.34" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="10 10"/>
  </asset>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" diffuse="0.7 0.7 0.7"/>
    <geom name="floor" type="plane" size="30 30 0.1" material="grid"/>
    {wall_xml}
    <body name="robot" pos="0 0 0.06">
      <geom name="chassis" type="box" size="0.16 0.10 0.05" rgba="0.25 0.55 0.90 1"/>
      <geom name="nose" type="box" pos="0.15 0 -0.02" size="0.03 0.06 0.02"
            rgba="0.95 0.95 0.95 1"/>
      <joint name="jx" type="slide" axis="1 0 0"/>
      <joint name="jy" type="slide" axis="0 1 0"/>
      <joint name="jyaw" type="hinge" axis="0 0 1"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="vx" joint="jx" kv="200" ctrlrange="-3 3"/>
    <velocity name="vy" joint="jy" kv="200" ctrlrange="-3 3"/>
    <velocity name="wz" joint="jyaw" kv="40" ctrlrange="-4 4"/>
  </actuator>
</mujoco>
"""
