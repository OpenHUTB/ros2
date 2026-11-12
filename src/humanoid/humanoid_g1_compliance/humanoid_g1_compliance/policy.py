"""LAC 策略推理封装（观测构建 + 启动状态机 + 刚度通道）。

上游把这套逻辑写在 ``inference.py`` 里，与 ``rclpy`` / ``unitree_hg`` 消息、
Xbox 手柄解析耦合在一起，无法脱离 ROS 运行。这里把它拆成纯 Python：

    JointState + Command  ->  ObsBuilder  ->  (1,950) + (1,6080)
                          ->  student actor  ->  action(23)
                          ->  q_target = action * scale + default

启动状态机与上游一致（IDLE -> NEUTRAL -> HANDOFF -> RAMP_UP -> POLICY），
只是把「按 A 键推进」换成了按时间自动推进，便于无头复现；``direct=True``
则直接以满刚度进入 POLICY（从训练站姿起步时最快、最稳）。
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .lac.action_map import action_to_q_target, make_scale_offset_tensors
from .lac.constants import (
    ART_TO_J23, DAMPING_23, DEFAULT_JOINT_POS_23, DEFAULT_JOINT_POS_ART,
    NUM_JOINTS, STIFFNESS_23,
)
from .lac.obs_builder import ObsBuilder
from .lac.policy_loader import load_policy, make_act_fn
from .sim_core import JointState

# ---- 启动阶段 PD 增益（与上游 inference.py 一致）----
KP_NEUTRAL = 35.0
KD_NEUTRAL = 1.5
DURATION_NEUTRAL = 3.0
DURATION_HANDOFF = 3.0
DURATION_RAMP_UP = 3.0

# ---- 默认指令 ----
DEFAULT_BASE_HEIGHT = 0.74
HEIGHT_MIN, HEIGHT_MAX = 0.56, 0.78
# 5 通道刚度：[K_lin_la, K_lin_ra, K_lin_to, K_ang_la, K_ang_ra]
DEFAULT_STIFFNESS_5 = (250.0, 250.0, 250.0, 55.0, 55.0)
STIFF_NAMES = ('K_lin_la', 'K_lin_ra', 'K_lin_to', 'K_ang_la', 'K_ang_ra')
STIFF_BAND = {'K_lin': (10.0, 500.0), 'K_ang': (10.0, 100.0)}

STATES = ('IDLE', 'NEUTRAL', 'HOLD_NEUTRAL', 'HANDOFF', 'HOLD_HANDOFF',
          'RAMP_UP', 'POLICY')


@dataclass
class Command:
    """策略指令（全部为策略训练时的量纲）。"""

    lin_vel: np.ndarray = field(default_factory=lambda: np.zeros(2))     # vx, vy
    ang_vel_yaw: np.ndarray = field(default_factory=lambda: np.zeros(1))
    waist_yaw: float = 0.0
    base_height: float = DEFAULT_BASE_HEIGHT
    ref_arm: np.ndarray = field(
        default_factory=lambda: np.asarray(DEFAULT_JOINT_POS_23[13:23], dtype=np.float64))
    stiffness: np.ndarray = field(
        default_factory=lambda: np.asarray(DEFAULT_STIFFNESS_5, dtype=np.float64))

    def clamp(self) -> 'Command':
        self.base_height = float(np.clip(self.base_height, HEIGHT_MIN, HEIGHT_MAX))
        self.stiffness = np.clip(self.stiffness, 1.0, 1000.0)
        return self


def resolve_weights(pkg_dir: Optional[str] = None) -> Tuple[str, str]:
    """返回 (权重路径, 配置路径)。"""
    here = os.path.dirname(os.path.abspath(__file__))
    root = pkg_dir or os.path.dirname(here)
    ckpt = os.path.join(root, 'weights', 'lac_g1_23dof.pt')
    cfg = os.path.join(root, 'config', 'lac_policy_cfg.yaml')
    if not os.path.isfile(ckpt):
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('humanoid_g1_compliance')
            ckpt = os.path.join(share, 'weights', 'lac_g1_23dof.pt')
            cfg = os.path.join(share, 'config', 'lac_policy_cfg.yaml')
        except Exception:  # noqa: BLE001
            pass
    return ckpt, cfg


def _load_ckpt_ascii(ckpt_path: str) -> str:
    """torch 的 C++ 反序列化不接受宽字符路径，含中文时先拷到临时目录。"""
    if not any(ord(c) > 127 for c in os.path.abspath(ckpt_path)):
        return ckpt_path
    import shutil
    import tempfile
    dst = os.path.join(tempfile.gettempdir(), 'lac_g1_23dof.pt')
    if not os.path.exists(dst) or os.path.getsize(dst) != os.path.getsize(ckpt_path):
        shutil.copyfile(ckpt_path, dst)
    return dst


class LacPolicy:
    """冻结的 LAC student actor + 观测历史。"""

    def __init__(self, weights: Optional[str] = None, cfg: Optional[str] = None,
                 device: str = 'cpu', verbose: bool = True):
        import torch
        self._torch = torch
        self.device = device

        if weights is None or cfg is None:
            w, c = resolve_weights()
            weights = weights or w
            cfg = cfg or c
        self.ckpt_path = weights
        self.cfg_path = cfg

        model, info = load_policy(_load_ckpt_ascii(weights), cfg, device=device)
        self.model = model
        self.act_fn = make_act_fn(model, device=device)
        self.info = info
        if info['num_policy_obs'] != 950 or info['num_policy_hist_obs'] != 6080:
            raise RuntimeError(
                f'权重维度不符（obs={info["num_policy_obs"]}, '
                f'hist={info["num_policy_hist_obs"]}），不是 LAC G1-23DOF 的 checkpoint？')
        if verbose:
            print(f'[LAC] 载入 {os.path.basename(weights)}  '
                  f'obs={info["num_policy_obs"]} hist={info["num_policy_hist_obs"]} '
                  f'device={device}')

        self.obs_builder = ObsBuilder(
            torch.tensor(DEFAULT_JOINT_POS_ART, dtype=torch.float32), device=device)
        self.scale_t, self.offset_t = make_scale_offset_tensors(device=device)
        self._art_idx = torch.tensor(ART_TO_J23, dtype=torch.long)
        self.last_action = torch.zeros(NUM_JOINTS, dtype=torch.float32)
        self.q_target = np.asarray(DEFAULT_JOINT_POS_23, dtype=np.float64)

    def reset(self) -> None:
        self.obs_builder.reset()
        self.last_action.zero_()

    def infer(self, state: JointState, cmd: Command) -> np.ndarray:
        """跑一步策略，返回 (23,) 的关节目标（JOINT_NAMES_23DOF 顺序）。"""
        t = self._torch
        q = t.tensor(state.q, dtype=t.float32)
        dq = t.tensor(state.dq, dtype=t.float32)
        quat = t.tensor(state.quat, dtype=t.float32)
        quat = quat / quat.norm().clamp(min=1e-9)
        gravity = t.tensor([[0.0, 0.0, -1.0]])
        from .lac.torch_utils import quat_rotate_inverse
        proj_g = quat_rotate_inverse(quat.unsqueeze(0), gravity).squeeze(0)

        obs950, hist = self.obs_builder.build_and_push(
            base_ang_vel=t.tensor(state.gyro, dtype=t.float32),
            projected_gravity=proj_g,
            cmd_lin_vel=t.tensor(cmd.lin_vel, dtype=t.float32),
            cmd_ang_vel_yaw=t.tensor(cmd.ang_vel_yaw, dtype=t.float32),
            cmd_waist_yaw=t.tensor([cmd.waist_yaw], dtype=t.float32),
            cmd_base_height=t.tensor([cmd.base_height], dtype=t.float32),
            ref_arm_joints=t.tensor(cmd.ref_arm, dtype=t.float32),
            dof_pos=q[self._art_idx],
            dof_vel=dq[self._art_idx],
            last_action=self.last_action,
            stiffness=t.tensor(cmd.stiffness, dtype=t.float32),
        )
        with t.inference_mode():
            action = self.act_fn(obs950, hist).squeeze(0)
        self.last_action = action.detach()
        self.q_target = action_to_q_target(
            action, self.scale_t, self.offset_t).cpu().numpy().astype(np.float64)
        return self.q_target

    def action(self) -> np.ndarray:
        return self.last_action.cpu().numpy().copy()


class StartupFSM:
    """启动状态机：按时间自动推进（替代上游的手柄按键）。

    NEUTRAL      : 从当前姿态 3 s 平滑过渡到默认站姿，kp 用低增益 35
    HANDOFF      : 3 s 把策略输出平滑混入，kp 仍为 35
    RAMP_UP      : 3 s 把 kp 从 35 升到训练刚度
    POLICY       : 满刚度跟踪策略输出
    """

    def __init__(self, direct: bool = True):
        self.direct = direct
        self.state = 'POLICY' if direct else 'IDLE'
        self.t_enter = 0.0
        self.q_from: Optional[np.ndarray] = None
        self._kp_full = np.asarray(STIFFNESS_23, dtype=np.float64)
        self._kd_full = np.asarray(DAMPING_23, dtype=np.float64)

    def reset(self) -> None:
        self.state = 'POLICY' if self.direct else 'IDLE'
        self.t_enter = 0.0
        self.q_from = None

    def step(self, now: float, q_now: np.ndarray,
             q_policy: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回 (q_cmd, kp, kd)。"""
        default = np.asarray(DEFAULT_JOINT_POS_23, dtype=np.float64)
        kp_low = np.full(NUM_JOINTS, KP_NEUTRAL)
        kd_low = np.full(NUM_JOINTS, KD_NEUTRAL)

        if self.direct:
            self.state = 'POLICY'
            return q_policy.copy(), self._kp_full.copy(), self._kd_full.copy()

        if self.state == 'IDLE':
            # 无外力时保持当前姿态（零力矩由调用方决定），进入 NEUTRAL 需显式 begin()
            return q_now.copy(), kp_low, kd_low

        if self.state == 'NEUTRAL':
            a = min((now - self.t_enter) / DURATION_NEUTRAL, 1.0)
            q_cmd = (1 - a) * self.q_from + a * default
            if a >= 1.0:
                self._goto('HOLD_NEUTRAL', now)
            return q_cmd, kp_low, kd_low

        if self.state == 'HOLD_NEUTRAL':
            return default.copy(), kp_low, kd_low

        if self.state == 'HANDOFF':
            a = min((now - self.t_enter) / DURATION_HANDOFF, 1.0)
            q_cmd = (1 - a) * self.q_from + a * q_policy
            if a >= 1.0:
                self._goto('HOLD_HANDOFF', now)
            return q_cmd, kp_low, kd_low

        if self.state == 'HOLD_HANDOFF':
            return q_policy.copy(), kp_low, kd_low

        if self.state == 'RAMP_UP':
            a = min((now - self.t_enter) / DURATION_RAMP_UP, 1.0)
            kp = (1 - a) * KP_NEUTRAL + a * self._kp_full
            if a >= 1.0:
                self._goto('POLICY', now)
            return q_policy.copy(), kp, self._kd_full.copy()

        return q_policy.copy(), self._kp_full.copy(), self._kd_full.copy()

    def _goto(self, state: str, now: float) -> None:
        self.state = state
        self.t_enter = now

    def begin(self, now: float, q_now: np.ndarray) -> None:
        """IDLE -> NEUTRAL。"""
        if self.state == 'IDLE':
            self.q_from = q_now.copy()
            self._goto('NEUTRAL', now)

    def advance(self, now: float, q_now: np.ndarray) -> None:
        """手动推进一个阶段（等价上游按 A）。"""
        if self.state == 'HOLD_NEUTRAL':
            self.q_from = q_now.copy()
            self._goto('HANDOFF', now)
        elif self.state == 'HOLD_HANDOFF':
            self._goto('RAMP_UP', now)


__all__ = [
    'LacPolicy', 'StartupFSM', 'Command', 'STATES', 'STIFF_NAMES', 'STIFF_BAND',
    'DEFAULT_STIFFNESS_5', 'DEFAULT_BASE_HEIGHT', 'HEIGHT_MIN', 'HEIGHT_MAX',
    'resolve_weights',
]
