# -*- coding: utf-8 -*-
"""MuJoCo 模型与旋转工具（**不导入 rclpy**）。

单独放一个模块的原因：``main.py`` 的终端模式要能在**没有 ROS** 的机器上跑，
如果它从 ``mujoco_sim`` 里 import MJCF，就会连带把 rclpy 拖进来。

世界系 z 向上（MuJoCo 默认，与 ROS ENU 一致），深度 = −z。
机体 0.457 × 0.338 × 0.254 m，质量由参数注入（默认 11 kg）。
"""

import math

import numpy as np

# 注意：gravity 设为 0 —— 重力由 dynamics.py 里的浮力/配重模型显式施加，
# 这样"净浮力"只有一个来源，避免与引擎自带重力重复计算。
MJCF = """
<mujoco model="rov_handling_eval">
  <compiler angle="degree" inertiafromgeom="true"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 0"/>
  <worldbody>
    <body name="rov" pos="0 0 0">
      <freejoint name="root"/>
      <geom name="hull" type="box" size="0.228 0.169 0.127" mass="{mass}"
            rgba="0.20 0.55 0.85 1.0"/>
    </body>
  </worldbody>
</mujoco>
"""


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """(w, x, y, z) → 3×3 旋转矩阵（机体系 → 世界系）。"""
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def build_model(mass: float = 11.0):
    """返回 (mujoco, model, data)，供 main.py 与 mujoco_sim 共用。"""
    import mujoco
    model = mujoco.MjModel.from_xml_string(MJCF.format(mass=mass))
    return mujoco, model, mujoco.MjData(model)
