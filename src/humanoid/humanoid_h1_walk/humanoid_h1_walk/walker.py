"""H1 / H1_2 行走策略的纯 Python 实现（不依赖 ROS）。

策略来自 Unitree 的 unitree_rl_gym（``deploy/pre_train``），是 Isaac Gym 里用
PPO 训练好的 TorchScript 文件。每个策略以 50 Hz 读取机器人状态（机身角速度、
重力方向、速度指令、腿部关节角与角速度、上一帧动作、步态相位），输出腿部关节
的目标角；PD 控制器再把目标角变成 10 个物理步（每步 2 ms）的力矩。

本模块把这套推理流程封装成 :class:`Walker`，供三处共用：

* ``main.py --demo``：不开窗口跑一遍固定动作序列，打印评价指标；
* ``scripts/``：无头测试、机型对比、录制动图；
* ``humanoid_h1_walk/sim_node.py`` + ``policy_node.py``：拆成两个 ROS 2 节点。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import mujoco
import numpy as np
import yaml

try:
    import torch
except ImportError as exc:  # pragma: no cover - 仅在环境缺依赖时触发
    raise ImportError(
        "humanoid_h1_walk 需要 PyTorch 才能加载 TorchScript 策略。\n"
        "请安装 CPU 版（约 200 MB，无需 CUDA）：\n"
        "  pip install torch --index-url https://download.pytorch.org/whl/cpu"
    ) from exc

#: 策略训练时的步态周期（秒），观测最后两维是它的 sin/cos 相位
GAIT_PERIOD = 0.8

#: 固定动作序列：(名称, 持续秒数, 前进速度 m/s, 转向角速度 rad/s)
ROUTINE: List[Tuple[str, float, float, float]] = [
    ("站立", 3.0, 0.0, 0.0),
    ("前进", 6.0, 0.5, 0.0),
    ("边走边左转", 6.0, 0.5, 0.5),
    ("前进", 4.0, 0.5, 0.0),
    ("原地右转", 5.0, 0.0, -0.6),
    ("后退", 4.0, -0.3, 0.0),
    ("停止", 3.0, 0.0, 0.0),
]


def package_root() -> Path:
    """包内资源（assets / weights / config）的根目录。

    安装后从 ``share/humanoid_h1_walk`` 取，源码目录里则回退到包的上两级。
    """
    override = os.environ.get("HUMANOID_H1_WALK_SHARE")
    if override:
        return Path(override)
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("humanoid_h1_walk"))
    except Exception:  # 未安装成 ROS 2 包时的源码运行路径
        return Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class RobotSpec:
    """一个机型跑策略所需的全部常量，对应 config/robots.yaml 的一项。"""

    name: str
    description: str
    model: Path
    weights: Path
    simulation_dt: float
    control_decimation: int
    num_obs: int
    num_actions: int
    kps: np.ndarray
    kds: np.ndarray
    default_angles: np.ndarray
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    action_scale: float
    cmd_scale: np.ndarray

    @property
    def control_dt(self) -> float:
        """控制周期（秒），即 2 ms × 10 = 20 ms，对应 50 Hz。"""
        return self.simulation_dt * self.control_decimation


def _as_array(value, dtype=np.float32) -> np.ndarray:
    return np.asarray(value, dtype=dtype)


def ascii_model_path(xml_path: Path) -> Path:
    """返回一个 MuJoCo 打得开的模型路径。

    MuJoCo 的 ``mj_loadXML`` 在 Windows 上按窄字符读文件名，绝对路径里只要含
    非 ASCII 字符（例如工作区在 ``D:\\机器人操作系统\\`` 下）就会报
    ``Error opening file``。这里把该机型的整个资产目录复制到临时目录（ASCII
    路径）后加载，只复制一次，后续运行直接命中缓存。

    相对路径不受影响，因为 MuJoCo 会按当前工作目录解析；但包资源一律用绝对
    路径定位（见 :func:`package_root`），所以这个兜底是必需的。
    """
    if str(xml_path).isascii():
        return xml_path

    import hashlib
    import shutil
    import tempfile

    robot_dir = xml_path.parent
    digest = hashlib.md5(str(robot_dir).encode("utf-8")).hexdigest()[:10]
    cache_dir = Path(tempfile.gettempdir()) / f"humanoid_h1_walk_{digest}" / robot_dir.name
    target = cache_dir / xml_path.name
    if not target.exists():
        shutil.copytree(robot_dir, cache_dir, dirs_exist_ok=True)
        print(f"[humanoid_h1_walk] 模型路径含非 ASCII 字符，已复制到 {cache_dir} 供 MuJoCo 加载")
    return target


def load_specs(config_path: Path | str | None = None) -> Dict[str, RobotSpec]:
    """读取 config/robots.yaml，返回 ``{机型名: RobotSpec}``。"""
    root = package_root()
    path = Path(config_path) if config_path else root / "config" / "robots.yaml"
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    specs: Dict[str, RobotSpec] = {}
    for name, item in raw["robots"].items():
        specs[name] = RobotSpec(
            name=name,
            description=item["description"],
            model=root / item["model"],
            weights=root / item["weights"],
            simulation_dt=float(item["simulation_dt"]),
            control_decimation=int(item["control_decimation"]),
            num_obs=int(item["num_obs"]),
            num_actions=int(item["num_actions"]),
            kps=_as_array(item["kps"]),
            kds=_as_array(item["kds"]),
            default_angles=_as_array(item["default_angles"]),
            ang_vel_scale=float(item["ang_vel_scale"]),
            dof_pos_scale=float(item["dof_pos_scale"]),
            dof_vel_scale=float(item["dof_vel_scale"]),
            action_scale=float(item["action_scale"]),
            cmd_scale=_as_array(item["cmd_scale"]),
        )
    return specs


def gravity_in_body(quat) -> np.ndarray:
    """把世界坐标下的重力方向旋转到机体系，得到策略的「姿态」输入。

    与 Unitree ``deploy_mujoco.py`` 的写法一致：四元数取 ``[w, x, y, z]``，
    返回机体系下单位重力向量 ``g_b = R^T · [0, 0, -1]``（这里省略了负号，
    与上游实现保持同一约定，策略也是按这个约定训练的）。
    """
    qw, qx, qy, qz = quat
    return np.array(
        [
            2.0 * (-qz * qx + qw * qy),
            -2.0 * (qz * qy + qw * qx),
            1.0 - 2.0 * (qw * qw + qz * qz),
        ]
    )


def routine_command(t: float) -> Tuple[np.ndarray, str]:
    """返回固定动作序列在时刻 ``t``（秒）的速度指令和阶段名称。"""
    total = sum(seconds for _, seconds, _, _ in ROUTINE)
    t = t % total
    for label, seconds, forward, turn in ROUTINE:
        if t < seconds:
            return np.array([forward, 0.0, turn], dtype=np.float32), label
        t -= seconds
    return np.zeros(3, dtype=np.float32), ROUTINE[-1][0]


def load_torchscript(path: Path):
    """加载 TorchScript 策略，兼容含非 ASCII 字符的路径。

    ``torch.jit.load`` 内部同样按窄字符 fopen，Windows 下遇到中文路径会报
    ``errno 2``；先用 Python 读成字节流再交给它，就绕开了这个问题。
    """
    import io

    return torch.jit.load(io.BytesIO(Path(path).read_bytes()), map_location="cpu")


class Walker:
    """一个机型 + 它对应的策略，负责「观测 → 动作 → PD → 物理」的闭环。"""

    def __init__(
        self,
        robot: str,
        specs: Dict[str, RobotSpec] | None = None,
        load_policy: bool = True,
    ):
        self.specs = specs or load_specs()
        if robot not in self.specs:
            raise KeyError(f"未知机型 {robot}，可选：{sorted(self.specs)}")
        self.spec = spec = self.specs[robot]

        self.model = mujoco.MjModel.from_xml_path(str(ascii_model_path(spec.model)))
        self.model.opt.timestep = spec.simulation_dt
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.policy = None
        if load_policy:
            self.policy = load_torchscript(spec.weights)
            self.policy.eval()

        self.action = np.zeros(spec.num_actions, dtype=np.float32)
        self.target = spec.default_angles.copy()
        self.obs = np.zeros(spec.num_obs, dtype=np.float32)
        self.control_dt = spec.control_dt

    # ---------- 观测 ----------
    def observe(self, cmd, action=None) -> np.ndarray:
        """拼出策略的输入向量。

        布局（共 ``9 + 3n + 2`` 维，n 为关节数）：:

            [0:3]   机身角速度 × 0.25
            [3:6]   机体系重力方向
            [6:9]   速度指令 × [2, 2, 0.25]
            [9:9+n] 关节角相对默认姿态的偏差
            [..]    关节角速度 × 0.05
            [..]    上一帧动作
            [..]    步态相位的 sin / cos
        """
        spec, d, n = self.spec, self.data, self.spec.num_actions
        a = self.action if action is None else action
        phase = (d.time % GAIT_PERIOD) / GAIT_PERIOD
        o = self.obs
        o[:3] = d.qvel[3:6] * spec.ang_vel_scale
        o[3:6] = gravity_in_body(d.qpos[3:7])
        o[6:9] = np.asarray(cmd, dtype=np.float32) * spec.cmd_scale
        o[9 : 9 + n] = (d.qpos[7:] - spec.default_angles) * spec.dof_pos_scale
        o[9 + n : 9 + 2 * n] = d.qvel[6:] * spec.dof_vel_scale
        o[9 + 2 * n : 9 + 3 * n] = a
        o[9 + 3 * n] = np.sin(2.0 * np.pi * phase)
        o[9 + 3 * n + 1] = np.cos(2.0 * np.pi * phase)
        return o

    # ---------- 动作 ----------
    def act(self, obs=None) -> np.ndarray:
        """跑一次策略推理，返回动作（未缩放，范围约 -3~3）。"""
        if self.policy is None:
            raise RuntimeError("Walker 以 load_policy=False 构造，无法推理")
        x = self.obs if obs is None else obs
        with torch.no_grad():
            out = self.policy(torch.from_numpy(np.asarray(x, dtype=np.float32)).unsqueeze(0))
        return out.squeeze(0).numpy().astype(np.float32)

    def apply_action(self, action) -> np.ndarray:
        """把动作缩放成关节目标角：``q* = a · 0.25 + q_default``。"""
        self.action = np.asarray(action, dtype=np.float32).copy()
        self.target = self.action * self.spec.action_scale + self.spec.default_angles
        return self.target

    # ---------- 物理 ----------
    def advance(self) -> None:
        """按当前目标角做 PD 控制，推进 ``control_decimation`` 个物理步。"""
        d, spec = self.data, self.spec
        for _ in range(spec.control_decimation):
            d.ctrl[:] = (self.target - d.qpos[7:]) * spec.kps - d.qvel[6:] * spec.kds
            mujoco.mj_step(self.model, d)

    def step(self, cmd) -> np.ndarray:
        """完整走一个控制周期：物理 → 观测 → 策略 → 新目标角。"""
        self.advance()
        obs = self.observe(cmd)
        self.apply_action(self.act(obs))
        return obs

    # ---------- 状态 ----------
    @property
    def base_height(self) -> float:
        """骨盆离地高度（米）。"""
        return float(self.data.qpos[2])

    @property
    def base_position(self) -> np.ndarray:
        return np.array(self.data.qpos[:3], dtype=np.float64)

    def base_lin_vel_body(self) -> np.ndarray:
        """机体系线速度。``qvel[0:3]`` 是世界系线速度，需用机身姿态旋转。"""
        qw, qx, qy, qz = self.data.qpos[3:7]
        rot = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
                [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
            ]
        )
        return rot.T @ self.data.qvel[:3]

    @property
    def yaw(self) -> float:
        qw, qx, qy, qz = self.data.qpos[3:7]
        return float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))

    @property
    def joint_names(self) -> List[str]:
        return [
            self.model.joint(i).name
            for i in range(self.model.njnt)
            if self.model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
        ]

    def reset(self) -> None:
        """复位物理状态，并把策略的 LSTM 隐状态清零。

        策略导出成 ``PolicyExporterLSTM``，``hidden_state`` / ``cell_state``
        是模块自身的张量，会跨帧保留（相当于 0.8 s 步态周期内的短期记忆）。
        重新开始一段仿真时必须清零，否则上一段的记忆会带进新的一段。
        """
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.action = np.zeros(self.spec.num_actions, dtype=np.float32)
        self.target = self.spec.default_angles.copy()
        if self.policy is not None:
            for attr in ("hidden_state", "cell_state"):
                state = getattr(self.policy, attr, None)
                if isinstance(state, torch.Tensor):
                    state.zero_()
