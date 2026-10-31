# -*- coding: utf-8 -*-
"""集中配置：路径、几何、控制参数、渲染后端引导。

渲染后端必须在 `import mujoco` 之前确定，所以任何用到 mujoco 的模块
都要先 import 本文件（`env` 已经这么做了）。
"""

import os
import sys
from pathlib import Path

# config.py 在 <包>/src/balance_bot/ 下，往上是三层才是包根目录
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

DEFAULT_SCENE = PACKAGE_ROOT / "models" / "balance_bot.xml"
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
CONTROL_HZ = 50.0        # 控制/话题频率（真实自平衡车一般 100~200 Hz，这里 50 够用）
SETTLE_SECONDS = 0.0     # 不做静置：倒立摆静置时自己就倒了

# ---------------------------------------------------------------------------
# 机器人几何（与 models/balance_bot.xml 严格对应）
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.05
WHEEL_Y = 0.15
TRACK = 2.0 * WHEEL_Y    # 轮距 0.30 m
# 出生高度 = 轮半径（**不是底座中心高度**）：设成 0.128 时轮子悬空 7.8 cm，
# 实测 ncon=0，车是掉下来卡在底座上，看起来像"倒了"其实是没接触
SPAWN_Z = WHEEL_RADIUS - 0.002
SPAWN_TILT = 0.02        # 初始倾角（rad）：完全竖直时数值上永远不动

# ---------------------------------------------------------------------------
# 控制参数
# ---------------------------------------------------------------------------
# 力矩上限（单个电机，N·m）。整个车 6 kg 左右，4 N·m 足够把它"蹬"回来
TORQUE_MAX = 4.0

# 平衡 PD（LQR 的简化版，第四步会用解析 LQR 增益替换／对照）
KP_PITCH = 45.0          # 倾角 → 力矩（网格搜索：28~60 都稳定，取中间值留裕度）
KD_PITCH = 2.0           # 角速度 → 力矩（>5 会引入抖动）
KV_SPEED = 2.6           # 速度误差 → 力矩
K_YAW = 0.9            # 转向：偏航角速度误差 → 差动力矩

V_MAX = 0.60             # 前进速度上限 (m/s)
W_MAX = 1.20             # 转向角速度上限 (rad/s)

# 倾角超过这个就认为"倒了"（rad，约 40°）
FALL_ANGLE = 0.35

# ---------------------------------------------------------------------------
# 神经网络数据采集
# ---------------------------------------------------------------------------
N_EPISODES = 40          # 采集多少段随机工况
EPISODE_SECONDS = 6.0
