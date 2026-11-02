"""MuJoCo 四旋翼环境：速度伺服 + 前视深度相机 + 三种障碍布局。

物理积分由 MuJoCo 完成（RK4，dt=5 ms）；本文件只做两件事：

1. **控制抽象**：把"期望速度 (vx, 偏航角速度)"变成世界坐标系下的推力与力矩。
   姿态内环简化为 PD 力矩伺服（把机体系 z 轴拉回世界 z 轴并跟踪偏航角速度），
   因此外环看到的是一个"速度可指令"的飞行器，而积分、重力、碰撞仍然是真物理。
2. **传感器**：用 MuJoCo 的离屏渲染拿 64×64 前视深度图，并给出左右中三个扇区的
   最近距离（监督标签）。

坐标系约定：世界系 x 向前、y 向左、z 向上；机头方向为世界系 x 轴旋转 yaw 后的方向。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    import mujoco
except ImportError as exc:  # pragma: no cover - 允许纯 numpy 测试跳过
    raise ImportError("需要 mujoco：pip install mujoco") from exc

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "uav_depth_scene.xml"

DEPTH_HW = 64          # 相机分辨率
D_MAX = 8.0            # 深度截断（米），超过此值视为"前方空旷"
DT = 0.05              # 控制周期 20 Hz
SUBSTEPS = int(round(DT / 0.005))

GRAVITY = 9.81


@dataclass
class State:
    pos: np.ndarray          # 世界坐标 (x, y, z)
    vel: np.ndarray          # 世界坐标线速度
    yaw: float               # 偏航角 (rad)
    yaw_rate: float          # 偏航角速度 (rad/s)
    crashed: bool


def yaw_of(quat: np.ndarray) -> float:
    """四元数 (w, x, y, z) -> 偏航角。"""
    w, x, y, z = quat
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class DepthUavEnv:
    """一个可复现的深度相机避障场景（无需 ROS、无需 GPU）。"""

    SCENES = ("slalom", "gate", "clutter")

    def __init__(self, scene: str = "slalom", seed: int = 0, dt: float = DT):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        self.substeps = int(round(dt / self.model.opt.timestep))
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "uav")
        self.post_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"post{i}")
                         for i in range(6)]
        self.wall_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
                         for name in ("wall_left", "wall_right", "end_wall")]
        self.renderer = mujoco.Renderer(self.model, DEPTH_HW, DEPTH_HW)
        self.renderer.enable_depth_rendering()
        self.mass = float(self.model.body_subtreemass[self.body_id])
        self.yaw_ref = 0.0
        self.scene = scene
        self.seed = seed
        self.reset(scene=scene, seed=seed)

    # ---------------------------------------------------------------- 场景与重置
    def set_scene(self, scene: str, seed: int = 0) -> None:
        """按场景名重新摆放六个柱子。同一个 XML 复用三种布局。"""
        if scene not in self.SCENES:
            raise ValueError(f"未知场景 {scene}，可选 {self.SCENES}")
        rng = np.random.default_rng(seed)
        positions = []
        if scene == "slalom":
            for i in range(6):
                positions.append((3.0 + 2.0 * i, 1.4 if i % 2 == 0 else -1.4, 1.5))
        elif scene == "gate":
            # 两组"门"：柱心 ±0.8 m，留 1.6 m 通道（机身宽 0.44 m，余量约 0.5 m）
            for i in range(6):
                x = 5.0 if i < 3 else 10.0
                y = (-0.8, 0.8, 4.0)[i % 3]
                positions.append((x, y, 1.5))
        else:  # clutter
            for _ in range(6):
                positions.append((float(rng.uniform(3.0, 13.0)),
                                  float(rng.uniform(-2.2, 2.2)), 1.5))
        for gid, (x, y, z) in zip(self.post_ids, positions):
            self.model.geom_pos[gid] = (x, y, z)
            # 门洞场景里多余的柱子挪到墙外，等价于"不存在"
            if y > 3.0:
                self.model.geom_pos[gid] = (x, 8.0, z)
        self.scene, self.seed = scene, seed
        self.refresh()

    def refresh(self) -> None:
        """模型里的 geom_pos 被改过后，必须重算一次正向运动学，否则渲染仍用旧位置。"""
        mujoco.mj_forward(self.model, self.data)

    def reset(self, scene: str | None = None, seed: int | None = None,
              pos=(0.0, 0.0, 1.5), yaw: float = 0.0) -> State:
        self.set_scene(scene or self.scene, self.seed if seed is None else seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = pos
        self.data.qpos[3:7] = (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))
        mujoco.mj_forward(self.model, self.data)
        self.yaw_ref = yaw
        self.crashed = False
        return self.state()

    # ------------------------------------------------------------------- 状态
    def state(self) -> State:
        q = self.data.qpos
        return State(
            pos=np.array(q[0:3]),
            vel=np.array(self.data.qvel[0:3]),
            yaw=yaw_of(q[3:7]),
            yaw_rate=float(self.data.qvel[5]),
            crashed=self.crashed,
        )

    # ------------------------------------------------------------------- 传感器
    def depth(self) -> np.ndarray:
        """前视深度图，裁剪到 D_MAX，单位米。"""
        self.renderer.update_scene(self.data, camera="depth_cam")
        depth = np.array(self.renderer.render(), dtype=np.float32)
        depth = np.where(np.isfinite(depth), depth, D_MAX)
        return np.clip(depth, 0.0, D_MAX)

    def rgb(self, camera: str = "chase_cam") -> np.ndarray:
        """彩色图，用于 GIF 与文档截图（不参与控制）。"""
        was_depth = True
        self.renderer.disable_depth_rendering()
        try:
            self.renderer.update_scene(self.data, camera=camera)
            frame = np.array(self.renderer.render(), dtype=np.uint8)
        finally:
            if was_depth:
                self.renderer.enable_depth_rendering()
        return frame

    # ------------------------------------------------------------------- 控制
    def step(self, vx: float, yaw_rate: float = 0.0):
        """以 20 Hz 下发 (前向速度, 偏航角速度)，内部积分子步。"""
        R = self.data.xmat[self.body_id].reshape(3, 3)
        yaw = yaw_of(self.data.qpos[3:7])
        v_des = np.array([vx * np.cos(yaw), vx * np.sin(yaw), 0.0])
        v = np.array(self.data.qvel[0:3])

        # 外环：速度误差 -> 期望加速度 -> 世界系推力（含重力补偿）
        a_des = 2.5 * (v_des - v)
        a_des[2] += GRAVITY
        force = np.clip(self.mass * a_des, (-25, -25, 0), (25, 25, 29))

        # 内环：姿态 PD（机体系 z 轴对齐世界 z）+ 偏航角 PD（跟踪积分后的 yaw_ref）
        z_body = R[:, 2]
        err = np.cross(z_body, np.array([0.0, 0.0, 1.0]))
        omega = R @ np.array(self.data.qvel[3:6])
        self.yaw_ref += yaw_rate * self.dt
        yaw_err = float(np.arctan2(np.sin(self.yaw_ref - yaw), np.cos(self.yaw_ref - yaw)))
        torque = np.array([3.0 * err[0] - 0.35 * omega[0],
                           3.0 * err[1] - 0.35 * omega[1],
                           2.5 * yaw_err - 0.9 * omega[2]])

        self.data.xfrc_applied[self.body_id, 0:3] = force
        self.data.xfrc_applied[self.body_id, 3:6] = np.clip(torque, -4.0, 4.0)

        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self._check_crash()
        return self.state()

    def _check_crash(self) -> None:
        for contact in self.data.contact[: self.data.ncon]:
            pair = (contact.geom1, contact.geom2)
            if self.body_id in (self.model.geom_bodyid[pair[0]], self.model.geom_bodyid[pair[1]]):
                self.crashed = True
                return

    # ------------------------------------------------------------------- 标签
    @staticmethod
    def sector_clearance(depth: np.ndarray) -> np.ndarray:
        """把深度图切成左/中/右三个扇区，各取最近距离（归一化到 [0,1]）。

        | 扇区 | 列范围 | 含义 |
        | --- | --- | --- |
        | 左  | 0 – 20 | 左侧是否可绕行 |
        | 中  | 22 – 42 | 正前方是否通畅 |
        | 右  | 44 – 63 | 右侧是否可绕行 |
        """
        h, w = depth.shape
        third = w // 3
        top = depth[: int(h * 0.62)]      # 忽略下方 38% 的地面像素，只统计前方的障碍
        left = float(top[:, : third - 1].min())
        mid = float(top[:, third + 1: 2 * third - 1].min())
        right = float(top[:, 2 * third + 1:].min())
        return np.array([left, mid, right], dtype=np.float32) / D_MAX
