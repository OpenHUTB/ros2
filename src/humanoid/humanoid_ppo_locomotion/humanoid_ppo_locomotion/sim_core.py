"""G1 人形机器人 MuJoCo 仿真核心。

模型来源：unitreerobotics/unitree_mujoco（BSD-3-Clause）中的
``unitree_robots/g1/scene_29dof.xml``。本文件只做二次封装，不修改原始模型。

.. warning::
   **必须加载 ``scene_29dof.xml``，不能加载 ``g1_29dof.xml``。**
   后者只是机器人本体，不含地面；用它仿真时机器人会一直自由落体，
   表现为「高度从 0.80 掉到 0.41、但姿态始终直立（up_z≈1.0）」，
   极易被误判成「站不稳」。``scene_29dof.xml`` 才 ``include`` 了本体
   并加了 ``<geom name="floor" type="plane"/>``。
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import mujoco
import numpy as np

# 12 个腿部关节，顺序与 MJCF 中 actuator 顺序一致
LEG_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

# 上半身 17 个关节（执行器名，无 _joint 后缀），顺序与模型执行器索引一致
UPPER_JOINTS = [
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]

# 每个腿关节的 PD 增益（kp, kd），与 G1 官方控制习惯量级一致
LEG_GAINS = {
    "hip_pitch": (100.0, 2.0),
    "hip_roll": (100.0, 2.0),
    "hip_yaw": (60.0, 1.5),
    "knee": (150.0, 3.0),
    "ankle_pitch": (40.0, 1.0),
    "ankle_roll": (40.0, 1.0),
}

# 站立标称姿态（弧度）：髋微屈、膝微屈、踝回正
NOMINAL_LEG_POSE = np.array(
    [
        -0.15, 0.0, 0.0, 0.30, -0.15, 0.0,   # 左腿
        -0.15, 0.0, 0.0, 0.30, -0.15, 0.0,   # 右腿
    ],
    dtype=np.float64,
)

# 上半身保持姿态的 PD 增益
UPPER_KP = 40.0
UPPER_KD = 1.5


def _gain_for(joint_name: str) -> tuple[float, float]:
    for key, gain in LEG_GAINS.items():
        if key in joint_name:
            return gain
    return (UPPER_KP, UPPER_KD)


@dataclass
class SimConfig:
    model_path: str
    timestep: float = 0.002
    frame_skip: int = 10          # 控制频率 50 Hz（实测低于 50 Hz 无法站稳）
    gravity: float = -9.81

    @property
    def dt_ctrl(self) -> float:
        return self.timestep * self.frame_skip


class G1Sim:
    """轻量 MuJoCo 封装：只负责物理步进、PD 跟踪与状态读取。"""

    def __init__(self, cfg: SimConfig):
        if not os.path.exists(cfg.model_path):
            raise FileNotFoundError(
                f"未找到 G1 模型：{cfg.model_path}\n"
                "请先运行 scripts/fetch_assets.py 下载模型资源。"
            )
        self.cfg = cfg
        self.model = mujoco.MjModel.from_xml_path(cfg.model_path)
        self.model.opt.timestep = cfg.timestep
        self.data = mujoco.MjData(self.model)

        # 关节名 -> 地址映射
        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]
        self.actuator_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(self.model.nu)
        ]

        self.leg_qpos_adr = np.array(
            [self.model.jnt_qposadr[self.model.joint(n).id] for n in LEG_JOINTS]
        )
        self.leg_dof_adr = np.array(
            [self.model.jnt_dofadr[self.model.joint(n).id] for n in LEG_JOINTS]
        )
        # 每个执行器对应的关节自由度
        self.act_dof_adr = np.array(
            [self.model.jnt_dofadr[self.model.actuator_trnid[i, 0]]
             for i in range(self.model.nu)]
        )
        self.act_qpos_adr = np.array(
            [self.model.jnt_qposadr[self.model.actuator_trnid[i, 0]]
             for i in range(self.model.nu)]
        )
        self.leg_act_ids = np.array(
            [self.model.actuator(n.replace("_joint", "")).id for n in LEG_JOINTS],
            dtype=int,
        )
        self.upper_act_ids = np.array(
            [i for i in range(self.model.nu) if i not in set(self.leg_act_ids.tolist())],
            dtype=int,
        )
        self.leg_kp = np.array([_gain_for(n)[0] for n in LEG_JOINTS])
        self.leg_kd = np.array([_gain_for(n)[1] for n in LEG_JOINTS])
        # 上半身保持增益（实例属性，便于部署时调硬以让手臂动作更明显）
        self.upper_kp = UPPER_KP
        self.upper_kd = UPPER_KD
        self.ctrl_lo = self.model.actuator_ctrlrange[:, 0].copy()
        self.ctrl_hi = self.model.actuator_ctrlrange[:, 1].copy()

        # 关节角限位直接从模型读，避免硬编码（ankle_roll 只有 ±0.262）
        self.leg_qmin = np.array(
            [self.model.jnt_range[self.model.joint(n).id][0] for n in LEG_JOINTS]
        )
        self.leg_qmax = np.array(
            [self.model.jnt_range[self.model.joint(n).id][1] for n in LEG_JOINTS]
        )
        self.upper_qmin = np.array(
            [self.model.jnt_range[self.model.joint(n + "_joint").id][0]
             for n in UPPER_JOINTS]
        )
        self.upper_qmax = np.array(
            [self.model.jnt_range[self.model.joint(n + "_joint").id][1]
             for n in UPPER_JOINTS]
        )

        self.n_act = len(LEG_JOINTS)
        # 脚部 body（用于质心相对脚的偏差，平移不变）
        self._foot_body_ids = [
            self.model.body(n).id for n in ("left_ankle_roll_link", "right_ankle_roll_link")
        ]
        # 质心速度缓存（在 step() 中更新一次）
        self._com_prev = None
        self._com_vel = np.zeros(3)
        self._com_err0 = 0.0
        self.reset()

    # ---------- 状态 ----------
    def reset(self, qpos_noise: float = 0.0) -> None:
        mujoco.mj_resetData(self.model, self.data)
        q = NOMINAL_LEG_POSE.copy()
        if qpos_noise > 0:
            q += np.random.uniform(-qpos_noise, qpos_noise, size=q.shape)
        self.data.qpos[2] = 0.80
        self.data.qpos[self.leg_qpos_adr] = q
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._com_prev = self.data.subtree_com[1].copy()
        self._com_vel = np.zeros(3)
        self._com_err0 = float(self.data.subtree_com[1][0] - self.foot_center_x())

    @property
    def base_height(self) -> float:
        return float(self.data.qpos[2])

    def projected_gravity(self) -> np.ndarray:
        """机体系下的重力方向，用于判断姿态是否倾倒。"""
        # 世界系 (0,0,-1) 旋转到机体系
        return self._body_rot().T @ np.array([0.0, 0.0, -1.0])

    def base_lin_vel(self) -> np.ndarray:
        # MuJoCo 的 free joint 前 6 个 dof 是基座线速度/角速度（世界系）
        return self.data.qvel[0:3].copy()

    def base_ang_vel(self) -> np.ndarray:
        return self.data.qvel[3:6].copy()

    def base_lin_vel_body(self) -> np.ndarray:
        """基座线速度，转到机体系（观测用，比世界系更通用）。"""
        return self._body_rot().T @ self.data.qvel[0:3]

    def _body_rot(self) -> np.ndarray:
        w, x, y, z = self.data.qpos[3:7]
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )

    def com(self) -> np.ndarray:
        """整机质心（世界系）。索引 1 是根 body 的子树。"""
        return self.data.subtree_com[1].copy()

    def com_vel(self) -> np.ndarray:
        """质心速度（世界系），由 step() 中按控制周期做有限差分得到。"""
        return self._com_vel.copy()

    def foot_center_x(self) -> float:
        """双脚中心的前向坐标（世界系）。"""
        return float(np.mean([self.data.xpos[b][0] for b in self._foot_body_ids]))

    def com_error(self) -> float:
        """质心相对双脚中心的前向偏差，**平移不变**，平衡基元的输入。

        用 ``(com_x − foot_x) − 初值`` 而不是 ``com_x − 常数``：后者是世界系
        绝对坐标，机器人一旦往前走就会爆炸，平衡器会去对抗前进而不是保持平衡。
        """
        return (self.com()[0] - self.foot_center_x()) - self._com_err0

    def joint_pos(self) -> np.ndarray:
        return self.data.qpos[self.leg_qpos_adr].copy()

    def joint_vel(self) -> np.ndarray:
        return self.data.qvel[self.leg_dof_adr].copy()

    # ---------- 控制 ----------
    def apply_joint_targets(
        self, targets: np.ndarray, upper_targets: np.ndarray | None = None
    ) -> None:
        """把 12 个腿部关节目标角转成 PD 力矩。

        ``upper_targets`` 为 17 个上半身关节的目标角（弧度，顺序同
        :data:`UPPER_JOINTS`）；为 ``None`` 时保持 0 位。
        """
        q = self.data.qpos[self.leg_qpos_adr]
        qd = self.data.qvel[self.leg_dof_adr]
        tau_leg = self.leg_kp * (targets - q) - self.leg_kd * qd

        ctrl = np.zeros(self.model.nu)
        ctrl[self.leg_act_ids] = np.clip(tau_leg, self.ctrl_lo[self.leg_act_ids],
                                        self.ctrl_hi[self.leg_act_ids])

        # 上半身：PD 跟踪给定姿态（默认 0 位）
        q_up = self.data.qpos[self.act_qpos_adr[self.upper_act_ids]]
        qd_up = self.data.qvel[self.act_dof_adr[self.upper_act_ids]]
        if upper_targets is None:
            tgt_up = np.zeros(len(self.upper_act_ids))
        else:
            tgt_up = np.clip(np.asarray(upper_targets, dtype=np.float64),
                             self.upper_qmin, self.upper_qmax)
        tau_up = self.upper_kp * (tgt_up - q_up) - self.upper_kd * qd_up
        ctrl[self.upper_act_ids] = np.clip(
            tau_up, self.ctrl_lo[self.upper_act_ids], self.ctrl_hi[self.upper_act_ids]
        )
        self.data.ctrl[:] = ctrl

    def step(
        self, joint_targets: np.ndarray, upper_targets: np.ndarray | None = None
    ) -> None:
        self.apply_joint_targets(joint_targets, upper_targets)
        for _ in range(self.cfg.frame_skip):
            mujoco.mj_step(self.model, self.data)
        # 刷新质心速度（一阶低通，抑制 50 Hz 差分的噪声）
        com = self.data.subtree_com[1]
        v = (com - self._com_prev) / self.cfg.dt_ctrl
        self._com_vel = 0.5 * self._com_vel + 0.5 * v
        self._com_prev = com.copy()

    # ---------- 渲染 ----------
    def make_renderer(self, width: int = 480, height: int = 360):
        return mujoco.Renderer(self.model, height, width)

    def render(self, renderer, camera: str | None = None) -> np.ndarray:
        if camera is not None:
            renderer.update_scene(self.data, camera=camera)
        else:
            # 跟随视角：看向机器人躯干（距离 2.2 m 能让上半身动作看得清）
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [self.data.qpos[0], self.data.qpos[1], 0.62]
            cam.distance = 2.5
            cam.azimuth = 135.0
            cam.elevation = -12.0
            renderer.update_scene(self.data, camera=cam)
        return renderer.render()
