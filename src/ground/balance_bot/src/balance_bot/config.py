# -*- coding: utf-8 -*-
"""集中配置：路径、几何、控制参数。

**渲染后端不在这里决定**（评审指出过这个问题）：
早期版本在 import 阶段就调 `ensure_render_backend()`，只要机器没有 DISPLAY
就强行把 `MUJOCO_GL` 设成 osmesa；而没装 `libosmesa6` 的机器上
`import mujoco` 会直接抛
    OSError: ('OSMesa: cannot open shared object file: ...')
于是连"根本不需要渲染"的自检脚本都跑不起来。
现在改成惰性：只在 `env.render()` 真正要渲染时才挑后端，且失败会自动回退。
"""

import os
from pathlib import Path

# config.py 在 <包>/src/balance_bot/ 下，往上是三层才是包根目录
PACKAGE_ROOT = Path(__file__).resolve().parent.parent.parent


def render_backend_candidates():
    """按优先级给出可尝试的 MUJOCO_GL 取值（供 env.render 逐个试）。"""
    forced = os.environ.get("MUJOCO_GL")
    if forced:
        return [forced]
    cands = ["default"]
    if not os.environ.get("DISPLAY"):
        # 无桌面：先试软件渲染，再试 EGL（很多容器里 EGL 反而可用）
        cands += ["osmesa", "egl"]
    return cands


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

# 平衡 PD 增益（手工整定，**没有解 Riccati 方程**）。
# 说明：PD 可看作 LQR 的对角近似，但本模块没有真正求 K = R^-1 B^T P；
# 早期注释写过"第四步会用解析 LQR 增益替换"，这一步没有做，注释已改掉。
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
