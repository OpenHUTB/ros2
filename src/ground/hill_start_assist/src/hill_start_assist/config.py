# -*- coding: utf-8 -*-
"""集中配置：路径、几何、控制增益、初始位姿。"""

import math
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_SCENE = PACKAGE_ROOT / "models" / "slope_car.xml"


def resolve_scene_path(scene=None):
    p = Path(scene) if scene else DEFAULT_SCENE
    p = p.expanduser()
    if not p.is_absolute():
        p = (Path.cwd() / p).resolve()
    return str(p)


# 几何（与 models/slope_car.xml 对应；4 轮两轴布局）
WHEEL_RADIUS = 0.08
WHEEL_X = 0.12            # 前后轴到车心
WHEEL_Y = 0.16            # 左右轮距一半
WHEEL_DROP = 0.02          # 轮子相对车身原点下移量（XML 里 pos z=-0.02）
# 因此车身原点离接触点的距离 = 半径 + 偏移 = 0.10，出生高度必须用它，
# 少算 0.02 会让车陷进地面（实测穿透 2.69 cm）。
MASS = 7.0                # 实测整备质量

# 坡度（场景里 slope 的 euler 是 -12°，车也要按同样的角度摆）
SLOPE_DEG = 12.0
GRAVITY = 9.81

# 保持控制：四轮轮角保持 PD + 坡度前馈（力矩平摊到 4 个轮）
KP_HOLD = 45.0
KD_HOLD = 3.0
TAU_MAX = 6.0                      # 单轮力矩上限 (N·m)
ACTUATOR_GEAR = 20.0               # motor 的 gear（ctrl = τ / gear）

# 落稳阶段（这段不控制，也不计入测量）
SETTLE_S = 0.4

# 判据
DRIFT_OK_M = 0.15                  # 有辅助时 3 s 内的位移上限
PITCH_TOL_DEG = 3.0                # 车身俯仰必须贴近坡度，超出就算"翻/塌"
