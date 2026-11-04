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
import os
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

# 整个进程只提示一次"GL 深度不可用"，否则 run_matrix 每建一个环境就刷一行
_DEGRADED_WARNED = [False]


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
        self.cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "depth_cam")
        # 有些 OpenGL 实现（VMware SVGA / llvmpipe 等软件渲染）缺少 ARB_clip_control，
        # MuJoCo 的深度渲染会静默返回 inf，此时自动切到光线投射（见 depth()）。
        # 设环境变量 UAV_FORCE_RAYCAST=1 可强制使用光线投射，便于在没有该问题的机器上复现客户机行为。
        self.use_raycasting = os.environ.get("UAV_FORCE_RAYCAST", "") == "1"
        self.ray_grid = 16
        self._ray_has_normal = True     # MuJoCo >= 3.2 的 mj_ray 多一个 normal 出参，3.1.x 没有
        self.degraded_warned = _DEGRADED_WARNED
        self.yaw_ref = 0.0
        self.scene = scene
        self.seed = seed
        self.reset(scene=scene, seed=seed)

    # ---------------------------------------------------------------- 场景与重置
    def set_scene(self, scene: str, seed: int = 0) -> None:
        """按场景名重新摆放六个柱子。同一个 XML 复用三种布局。

        `seed` 会给出**同一个场景族内的不同实例**：slalom / gate 的柱子在基准位置上做
        随机偏移、门的宽度也在 ±0.15 m 内变化。这样"多随机种子"测的才是稳定性，
        而不是把同一条轨迹重复跑 5 次。
        """
        if scene not in self.SCENES:
            raise ValueError(f"未知场景 {scene}，可选 {self.SCENES}")
        rng = np.random.default_rng(seed)
        jitter = 0.0 if seed == 0 else 0.25      # seed=0 保持标称布局，便于对齐文档里的示意图
        positions = []
        if scene == "slalom":
            for i in range(6):
                dy = rng.normal(0, jitter) if jitter else 0.0
                positions.append((3.0 + 2.0 * i + rng.normal(0, jitter / 3),
                                  (1.4 if i % 2 == 0 else -1.4) + dy, 1.5))
        elif scene == "gate":
            half = 0.8 + (rng.uniform(-0.15, 0.15) if jitter else 0.0)
            for i in range(6):
                x = 5.0 if i < 3 else 10.0
                y = (-half, half, 4.0)[i % 3]
                positions.append((x + rng.normal(0, jitter / 3), y, 1.5))
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
        """前视深度图，裁剪到 D_MAX，单位米。

        先尝试 MuJoCo 的 GL 深度渲染；若结果是"全 inf / 全 D_MAX"（软件 OpenGL 缺少
        `ARB_clip_control` 时的典型表现），自动切到 `mj_ray` 光线投射，并在日志里警告一次。
        """
        if not self.use_raycasting:
            self.renderer.update_scene(self.data, camera="depth_cam")
            raw = np.array(self.renderer.render(), dtype=np.float32)
            if self._depth_looks_valid(raw):
                depth = np.where(np.isfinite(raw), raw, D_MAX)
                return np.clip(depth, 0.0, D_MAX)
            self.use_raycasting = True
            if not self.degraded_warned[0]:
                self.degraded_warned[0] = True
                print("[uav_nav] GL 深度渲染不可用（软件 OpenGL 缺少 ARB_clip_control），"
                      "已切换到 mj_ray 光线投射（16x16 采样后上采样）")
        return self.ray_depth()

    @staticmethod
    def _depth_looks_valid(raw: np.ndarray) -> bool:
        finite = np.isfinite(raw)
        if finite.mean() < 0.5:          # 一半以上是非有限值 -> 渲染失败
            return False
        valid = raw[finite]
        if valid.size and np.all(valid >= D_MAX * 0.999):
            return False                 # 全是最远距离 -> 没有真正渲染出几何
        return True

    def _cast_ray(self, pnt, vec, geomid, normal) -> float:
        """兼容不同 MuJoCo 版本的 `mj_ray` 签名（3.1.x 只有 8 个参数）。"""
        if self._ray_has_normal:
            try:
                return mujoco.mj_ray(self.model, self.data, pnt, vec, None, True,
                                     self.body_id, geomid, normal)
            except TypeError:
                self._ray_has_normal = False
        return mujoco.mj_ray(self.model, self.data, pnt, vec, None, True,
                             self.body_id, geomid)

    def ray_depth(self) -> np.ndarray:
        """用 `mj_ray` 逐像素投射得到深度图（不依赖 GL），再上采样到 64x64。"""
        g = self.ray_grid
        cam_pos = np.array(self.data.cam_xpos[self.cam_id], dtype=np.float64)
        R = np.array(self.data.cam_xmat[self.cam_id], dtype=np.float64).reshape(3, 3)
        # 相机看向 -z_cam；fovy=80°，方形视场
        scale = np.tan(np.deg2rad(80.0) / 2.0)
        grid = np.zeros((g, g), dtype=np.float32)
        geomid = np.zeros(1, dtype=np.int32)
        normal = np.zeros(3, dtype=np.float64)
        for i in range(g):
            v = (i + 0.5) / g * 2.0 - 1.0          # -1..1
            for j in range(g):
                u = (j + 0.5) / g * 2.0 - 1.0
                direction = R @ np.array([u * scale, -v * scale, -1.0])
                direction /= np.linalg.norm(direction)
                dist = self._cast_ray(cam_pos, direction, geomid, normal)
                grid[i, j] = D_MAX if dist < 0 else min(float(dist), D_MAX)
        step = DEPTH_HW // g
        return np.repeat(np.repeat(grid, step, axis=0), step, axis=1)

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
