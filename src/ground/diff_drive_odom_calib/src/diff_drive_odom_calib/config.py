# -*- coding: utf-8 -*-
"""集中配置：路径、机器人几何、仿真参数、以及 MuJoCo 渲染后端引导。

渲染后端必须在 `import mujoco` **之前**确定，所以任何用到 mujoco 的模块
都要先 import 本文件（`mujoco_env` 已经这么做了）。
"""

import os
import sys
from pathlib import Path

# config.py 在 <包>/src/diff_drive_odom_calib/ 下，往上是三层才是包根目录
PACKAGE_ROOT = Path(__file__).resolve().parent.parent.parent


def ensure_render_backend(verbose=False):
    """无图形界面的 Linux 上自动选 osmesa 软件渲染。

    虚拟机里没有 GPU 时 `mujoco.Renderer` 会抛
    `FatalError: an OpenGL platform library has not been loaded`；
    这个环境变量必须在 import mujoco 之前设置，import 之后再设无效。
    """
    if os.environ.get("MUJOCO_GL"):
        return os.environ["MUJOCO_GL"]
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        os.environ["MUJOCO_GL"] = "osmesa"
        if verbose:
            print("[渲染] 无 DISPLAY，使用 MUJOCO_GL=osmesa")
        return "osmesa"
    return "default"


ensure_render_backend()

# ---------------------------------------------------------------------------
DEFAULT_SCENE = PACKAGE_ROOT / "models" / "diff_drive.xml"
DEFAULT_OUTPUT_DIR = PACKAGE_ROOT / "results"


def resolve_scene_path(scene=None):
    p = Path(scene) if scene else DEFAULT_SCENE
    p = p.expanduser()
    if not p.is_absolute():
        p = (Path.cwd() / p).resolve()
    return str(p)


# ---------------------------------------------------------------------------
# 仿真
# ---------------------------------------------------------------------------
CONTROL_HZ = 50.0          # 控制/话题频率
SETTLE_SECONDS = 0.60      # 复位后静置，让接触稳定

# ---------------------------------------------------------------------------
# 机器人几何（与 models/diff_drive.xml 严格对应）
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.10
WHEEL_HALF_WIDTH = 0.03
WHEEL_Y = 0.18
TRACK = 2.0 * WHEEL_Y      # 轮距 0.36 m（标称值，可能不真实）
SPAWN_Z = WHEEL_RADIUS - 0.002

V_MAX = 0.80
W_MAX = 1.60

# ---------------------------------------------------------------------------
# 标定实验参数
# ---------------------------------------------------------------------------
# 里程计"以为"的参数：与真值（0.1000 / 0.360）差 3.5% 和 2.2%，
# 这是真实机器人出厂标定的典型量级（不是 2 倍这种离谱值）
R_NOMINAL = 0.1035
B_NOMINAL = 0.352

STRAIGHT_DISTANCE = 2.0
STRAIGHT_SPEED = 0.30
ROTATION_TURNS = 3.0
ROTATION_SPEED = 1.00
SQUARE_SIDE = 4.0
