"""G1 速度跟踪行走环境（Gymnasium 接口）。

架构：解析平衡基元 + PPO 残差
-----------------------------
动作 **不是**直接的关节目标角，而是叠加在平衡基元输出上的**残差偏置**::

    targets = ComBalance.targets(NOMINAL, com_err, com_vx) + RESIDUAL_SCALE * a

为什么这么设计：实测纯 PPO（动作直接给目标角）训练 1.5M 步后，确定性策略只能站
2.69 s（随机采样 6.22 s），而解析 CoM 反馈能站 >10 s。纯 PPO 打不过解析解，因为
倒立摆本身是不稳定平衡点，随机探索在 1.2 s 内就结束回合，学不到东西。加上稳定
基元后回合变长，PPO 才有时间学迈步。

基元负责「不摔倒」，NN 负责「怎么走」——NN 摆动腿使双脚中心前移，
``com_err`` 变负，基元自动把身体前倾跟上，这就是行走的力学来源。

观测 40 维::

    投影重力 3 + 基座线速度(机体系) 3 + 基座角速度 3
    + 高度偏差 1 + 12 关节相对标称角 + 12 关节角速度
    + 速度指令 2 + 步态相位 sin/cos 2 + 质心偏差 1 + 质心速度 1

动作 12 维：腿部关节目标角的残差偏置（乘 ``RESIDUAL_SCALE`` 后叠加）。
"""

from __future__ import annotations

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from .balance import ComBalance
from .sim_core import G1Sim, SimConfig, NOMINAL_LEG_POSE

OBS_DIM = 40
ACT_DIM = 12

# 残差幅度：0.35 rad ≈ 20°，足够抬腿迈步，又不至于把基元的平衡彻底覆盖
RESIDUAL_SCALE = 0.35

EPISODE_STEPS = 1000          # 50 Hz × 20 s
TARGET_HEIGHT = 0.78
MIN_HEIGHT = 0.45
MAX_TILT = 0.75               # 投影重力 xy 模长阈值


class G1WalkEnv(gym.Env):
    """速度指令跟踪环境。cmd = (vx, wz)。

    Parameters
    ----------
    model_path:
        **必须**是 ``scene_29dof.xml``（含地面），不能是 ``g1_29dof.xml``。
    use_balance:
        ``True``（默认）用平衡基元 + 残差；``False`` 退化成纯 PPO 直接输出目标角
        （仅用于对比实验，实测站不住）。
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 50}

    def __init__(
        self,
        model_path: str,
        cmd_vx_range: tuple[float, float] = (-0.3, 1.0),
        cmd_wz_range: tuple[float, float] = (-0.8, 0.8),
        episode_steps: int = EPISODE_STEPS,
        render_mode: str | None = None,
        use_balance: bool = True,
        residual_scale: float = RESIDUAL_SCALE,
    ):
        super().__init__()
        self.sim = G1Sim(SimConfig(model_path=model_path))
        self.cmd_vx_range = cmd_vx_range
        self.cmd_wz_range = cmd_wz_range
        self.episode_steps = episode_steps
        self.render_mode = render_mode
        self.use_balance = use_balance
        self.residual_scale = float(residual_scale)
        self.balance = ComBalance(
            q_min=self.sim.leg_qmin, q_max=self.sim.leg_qmax
        )

        self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)

        self._cmd = np.zeros(2, dtype=np.float64)
        self._phase = 0.0
        self._step = 0
        self._prev_action = np.zeros(ACT_DIM)
        self._renderer = None

    # ---------- 工具 ----------
    def _sample_cmd(self) -> np.ndarray:
        # cmd_scale 是课程学习系数：从 0 逐步放大到 1
        s = float(getattr(self, "cmd_scale", 1.0))
        return np.array(
            [
                np.random.uniform(*self.cmd_vx_range) * s,
                np.random.uniform(*self.cmd_wz_range) * s,
            ]
        )

    def base_targets(self) -> np.ndarray:
        """平衡基元给出的 12 维基准目标角（不含残差）。"""
        if not self.use_balance:
            return NOMINAL_LEG_POSE.copy()
        return self.balance.targets(
            NOMINAL_LEG_POSE,
            self.sim.com_error(),
            float(self.sim.com_vel()[0]),
        )

    def _obs(self) -> np.ndarray:
        pg = self.sim.projected_gravity()
        lin = self.sim.base_lin_vel_body()
        ang = self.sim.base_ang_vel()
        q = self.sim.joint_pos() - NOMINAL_LEG_POSE
        qd = self.sim.joint_vel()
        phase = np.array([np.sin(2 * np.pi * self._phase), np.cos(2 * np.pi * self._phase)])
        height = np.array([self.sim.base_height - TARGET_HEIGHT])
        com = np.array([self.sim.com_error(), float(self.sim.com_vel()[0])])
        return np.concatenate(
            [pg, lin, ang, height, q, qd, self._cmd, phase, com]
        ).astype(np.float32)

    def _reward(self, action: np.ndarray) -> tuple[float, dict]:
        lin = self.sim.base_lin_vel_body()
        ang = self.sim.base_ang_vel()
        pg = self.sim.projected_gravity()
        h = self.sim.base_height
        com_err = self.sim.com_error()

        vx_err = (lin[0] - self._cmd[0]) ** 2
        wz_err = (ang[2] - self._cmd[1]) ** 2

        r_lin = 1.5 * np.exp(-vx_err / 0.25)
        r_ang = 0.8 * np.exp(-wz_err / 0.25)
        r_up = 0.5 * np.exp(-5.0 * float(np.sum(pg[:2] ** 2)))
        r_h = 0.5 * np.exp(-20.0 * (h - TARGET_HEIGHT) ** 2)
        r_alive = 0.2
        r_energy = -1e-4 * float(np.sum(self.sim.data.ctrl ** 2))
        r_smooth = -0.01 * float(np.sum((action - self._prev_action) ** 2))
        r_lat = -0.5 * abs(float(lin[1]))          # 抑制侧向漂移
        r_com = -0.3 * com_err ** 2                # 质心别偏出脚掌

        total = (r_lin + r_ang + r_up + r_h + r_alive
                 + r_energy + r_smooth + r_lat + r_com)
        parts = dict(
            lin=float(r_lin), ang=float(r_ang), upright=float(r_up),
            height=float(r_h), energy=float(r_energy), smooth=float(r_smooth),
            lateral=float(r_lat), com=float(r_com),
        )
        return float(total), parts

    # ---------- Gym API ----------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.sim.reset(qpos_noise=0.02)
        self._cmd = (
            self._sample_cmd()
            if options is None or "cmd" not in options
            else np.asarray(options["cmd"], dtype=np.float64)
        )
        self._phase = 0.0
        self._step = 0
        self._prev_action = np.zeros(ACT_DIM)
        return self._obs(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        targets = self.base_targets() + self.residual_scale * action
        targets = np.clip(targets, self.sim.leg_qmin, self.sim.leg_qmax)
        self.sim.step(targets)

        self._step += 1
        self._phase = (self._phase + 0.02) % 1.0

        reward, parts = self._reward(action)
        self._prev_action = action

        pg = self.sim.projected_gravity()
        terminated = bool(
            self.sim.base_height < MIN_HEIGHT
            or float(np.linalg.norm(pg[:2])) > MAX_TILT
        )
        truncated = self._step >= self.episode_steps
        info = {"reward_parts": parts, "cmd": self._cmd.copy()}
        return self._obs(), reward, terminated, truncated, info

    def render(self):
        if self._renderer is None:
            self._renderer = self.sim.make_renderer()
        return self.sim.render(self._renderer)

    def close(self):
        self._renderer = None
