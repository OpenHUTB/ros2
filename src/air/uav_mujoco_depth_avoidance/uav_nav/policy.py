"""闭环策略：神经网络输出三个扇区通行度，控制律据此决定速度与偏航角速度。

控制律（$c_l, c_m, c_r \\in [0,1]$ 为网络预测的通行度）：

$$
v_x = v_{max}\\cdot \\mathrm{clip}\\!\\left(\\frac{c_m - c_{safe}}{1 - c_{safe}}, 0.05, 1\\right)
$$

$$
\\dot\\psi = k_\\psi (c_l - c_r) + k_{esc}\\,\\mathbb{1}[c_m < c_{safe}]\\,\\operatorname{sign}(c_l - c_r)
$$

- 前向速度随"正前方余量"线性下降：正前方越挤越慢；
- 偏航角速度朝向更空的一侧，$k_\\psi$ 决定常规绕行强度；
- 正前方余量低于安全阈值时叠加逃逸项 $k_{esc}$，避免贴脸才转向。

注意：控制律里的 $c$ **全部来自网络预测**，不读取真值深度；真值只用于评价。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .data import to_net_input
from .env import D_MAX, DepthUavEnv


@dataclass
class PolicyConfig:
    v_max: float = 2.4         # m/s
    c_safe: float = 0.14       # 低于此通行度视为"正前方不安全"
    c_nom: float = 0.5         # 通行度达到此值即允许全速
    k_psi: float = 1.5         # 通行度差 -> 偏航角速度
    k_head: float = 1.2        # 朝向回正增益（抑制绕圈）
    k_esc: float = 1.4         # 逃逸转向增益
    w_max: float = 1.5         # 偏航角速度上限 rad/s
    steps: int = 420           # 单次飞行最大控制步数（20 Hz -> 21 s）


@dataclass
class RunResult:
    scene: str
    success: bool
    crashed: bool
    steps: int
    traveled_x: float
    min_clearance: float
    mean_pred_error: float
    mean_speed: float
    path_length: float
    frames: list = field(default_factory=list)


def act(net, depth: np.ndarray, cfg: PolicyConfig, yaw: float = 0.0):
    """单步决策：深度图 + 当前偏航角 -> (vx, yaw_rate, 网络预测的通行度)。

    偏航指令由三部分叠加：

    $$
    \\dot\\psi = k_\\psi(c_l - c_r) - k_{head}\\psi + k_{esc}\\mathbb{1}[c_m< c_{safe}]\\operatorname{sign}(c_l-c_r)
    $$

    最后一项 $-k_{head}\\psi$ 是朝向回正项：没有它，只要左右通行度存在固定差，
    飞机就会一直朝同一方向转、绕圈而不前进（这是调参时实测踩到的坑）。
    """
    x = to_net_input(depth)[None, :, :, None]
    c = net.predict(x)[0]
    c_mid = float(c[1])
    span = max(1e-6, cfg.c_nom - cfg.c_safe)
    vx = cfg.v_max * float(np.clip((c_mid - cfg.c_safe) / span, 0.12, 1.0))
    yaw = float(np.arctan2(np.sin(yaw), np.cos(yaw)))
    yaw_rate = cfg.k_psi * (float(c[0]) - float(c[2])) - cfg.k_head * yaw
    if c_mid < cfg.c_safe:
        yaw_rate += cfg.k_esc * float(np.sign(float(c[0]) - float(c[2])) or 1.0)
    yaw_rate = float(np.clip(yaw_rate, -cfg.w_max, cfg.w_max))
    return vx, yaw_rate, c


def run_episode(net, scene: str, seed: int = 0, cfg: PolicyConfig | None = None,
                record: bool = False, env: DepthUavEnv | None = None) -> RunResult:
    """在一个场景里跑一次完整飞行，返回指标（可选返回 GIF 帧）。"""
    cfg = cfg or PolicyConfig()
    env = env or DepthUavEnv(scene, seed=seed)
    env.reset(scene=scene, seed=seed)
    pos_prev = env.state().pos.copy()
    path_len = 0.0
    min_clear = 1.0
    errors = []
    speeds = []
    frames = []
    success = False
    crashed = False

    for step in range(cfg.steps):
        depth = env.depth()
        state_now = env.state()
        vx, yaw_rate, c = act(net, depth, cfg, yaw=state_now.yaw)
        truth = env.sector_clearance(depth)
        errors.append(float(np.abs(c - truth).mean()))
        min_clear = min(min_clear, float(truth.min()))
        speeds.append(abs(vx))

        if record and step % 2 == 0:
            frames.append((env.rgb("chase_cam"), depth,
                           f"step {step:>3}  vx={vx:.2f}  yaw={yaw_rate:+.2f}  "
                           f"c=({c[0]:.2f},{c[1]:.2f},{c[2]:.2f})"))

        state = env.step(vx, yaw_rate)
        path_len += float(np.linalg.norm(state.pos - pos_prev))
        pos_prev = state.pos.copy()
        crashed = state.crashed
        if crashed:
            break
        if state.pos[0] > 13.6:
            success = True
            break

    return RunResult(
        scene=scene,
        success=success,
        crashed=crashed,
        steps=step + 1,
        traveled_x=float(pos_prev[0]),
        min_clearance=round(min_clear, 3),
        mean_pred_error=round(float(np.mean(errors)), 4),
        mean_speed=round(float(np.mean(speeds)), 3),
        path_length=round(path_len, 2),
        frames=frames,
    )


def run_all(net, seeds=(0, 1), cfg: PolicyConfig | None = None, log=print) -> list[RunResult]:
    """三种场景各跑若干个随机种子，便于做多场景性能评价。"""
    results = []
    for scene in DepthUavEnv.SCENES:
        for seed in seeds:
            res = run_episode(net, scene, seed=seed, cfg=cfg)
            results.append(res)
            log(f"[{scene}/seed={seed}] 成功={res.success} 碰撞={res.crashed} "
                f"步数={res.steps} 前进={res.traveled_x:.2f} m "
                f"最小余量={res.min_clearance:.3f} 预测误差={res.mean_pred_error:.3f} "
                f"路径长={res.path_length:.2f} m")
    return results
