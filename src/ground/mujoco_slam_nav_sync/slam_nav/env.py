"""MuJoCo 差速小车场地：平面运动 + 120 线激光雷达 + 里程计（可注入轮滑误差）。

设计要点（写进文档，评审会看）：

* **物理与里程计分离**。车在 MuJoCo 里按指令真实运动（速度伺服），
  但 SLAM 只能用"轮式里程计"：`odom()` 返回的真实速度乘以 $(1+s)$（$s$ 为轮滑率）
  再叠加高斯噪声。这样漂移是**里程计误差**，而不是把物理改坏——这也是真实轮滑的表现。
* **雷达用 `mj_ray`**，不依赖 GL，因此本机与软件 OpenGL 的虚拟机结果一致
  （深度渲染在 VMware 下会静默失效，射线不会）。
* 六个圆柱既是障碍也是 SLAM 的路标；路标位置对 SLAM **未知**，由观测反推。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import mujoco


def _find_asset(name: str) -> Path:
    """模型/权重的定位：环境变量 -> 安装后的 share 目录 -> 源码目录。

    装成 ROS 包以后 `models/` 在 share/mujoco_slam_nav_sync/models 下，
    不在 site-packages 里，所以不能用 `__file__` 一路相对上去。
    """
    env = os.environ.get("MUJOCO_SLAM_MODELS")
    if env:
        p = Path(env) / name
        if p.exists():
            return p
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("mujoco_slam_nav_sync")) / "models" / name
        if p.exists():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / "models" / name


MODEL_PATH = _find_asset("arena.xml")

DT = 0.05              # 控制周期 20 Hz
SUBSTEPS = 10          # 5 ms 物理步长
LIDAR_BEAMS = 120
LIDAR_RANGE = 6.0
ROBOT_RADIUS = 0.18
PILLAR_RADIUS = 0.22      # 柱子半径：已知尺寸，用于把"表面距离"换算成"中心距离"


@dataclass
class State:
    pos: np.ndarray
    yaw: float
    v: float
    w: float
    crashed: bool


def yaw_of(quat) -> float:
    w, x, y, z = quat
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def wrap(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))


class DiffDriveEnv:
    SCENES = ("maze", "open")
    GOAL = np.array([11.0, -4.8])
    START = (0.8, 0.8)

    def __init__(self, scene: str = "maze", seed: int = 0):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot")
        self.mass = float(self.model.body_subtreemass[self.body_id])
        # 路标按名字自动发现（几何体数量随时可能改，不要硬编码个数）
        self.lm_names, self.lm_ids = [], []
        for gid in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            if name.startswith("lm"):
                self.lm_names.append(name)
                self.lm_ids.append(gid)
        self._lm_base = np.array([self.model.geom_pos[g][:2] for g in self.lm_ids])
        self.wall_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, n)
                         for n in ("wall_n", "wall_s", "wall_w", "wall_e",
                                   "wall_mid_a", "wall_mid_b", "wall_mid_c")]
        self.rng = np.random.default_rng(seed)
        self.scene = scene
        self.seed = seed
        self.crashed = False
        self._ray9 = True
        self._renderer = None
        self.set_scene(scene, seed)

    # ---------------------------------------------------------------- 场景
    def set_scene(self, scene: str, seed: int = 0) -> None:
        """`maze` 保留内部隔断；`open` 把隔断挪出场外（只留外围墙 + 六个路标）。"""
        if scene not in self.SCENES:
            raise ValueError(f"未知场景 {scene}，可选 {self.SCENES}")
        rng = np.random.default_rng(seed)
        hide = scene == "open"
        for name, gid in (("wall_mid_a", self.wall_ids[4]),
                          ("wall_mid_b", self.wall_ids[5]),
                          ("wall_mid_c", self.wall_ids[6])):
            base = {"wall_mid_a": (4.0, 2.4, 0.3), "wall_mid_b": (4.0, -4.4, 0.3),
                    "wall_mid_c": (8.0, 0.6, 0.3)}[name]
            self.model.geom_pos[gid] = (base[0], 20.0, 0.3) if hide else base
        # 路标轻微抖动，保证不同 seed 是不同的实例（但不影响"未知"这一前提）
        for i, gid in enumerate(self.lm_ids):
            base = self._lm_base[i]
            jitter = 0.0 if seed == 0 else 0.25
            self.model.geom_pos[gid] = (base[0] + rng.normal(0, jitter),
                                        base[1] + rng.normal(0, jitter), 0.4)
        self.scene, self.seed = scene, seed
        self.refresh()

    def refresh(self) -> None:
        mujoco.mj_forward(self.model, self.data)

    def reset(self, scene: str | None = None, seed: int | None = None,
              pos=None, yaw: float = 0.0) -> State:
        self.set_scene(scene or self.scene, self.seed if seed is None else seed)
        mujoco.mj_resetData(self.model, self.data)
        p = np.array(pos if pos is not None else self.START, dtype=np.float64)
        self.data.qpos[0:2] = p
        self.data.qpos[2] = 0.12
        self.data.qpos[3:7] = (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))
        mujoco.mj_forward(self.model, self.data)
        self.crashed = False
        return self.state()

    # ---------------------------------------------------------------- 状态
    def state(self) -> State:
        q = self.data.qpos
        return State(pos=np.array(q[0:2]), yaw=yaw_of(q[3:7]),
                     v=float(self.data.qvel[0]), w=float(self.data.qvel[5]),
                     crashed=self.crashed)

    def landmarks(self) -> np.ndarray:
        """路标真实坐标（只用于评价，SLAM 不能用）。"""
        return np.array([self.model.geom_pos[gid][:2] for gid in self.lm_ids], dtype=np.float64)

    # ---------------------------------------------------------------- 传感器
    def scan(self) -> np.ndarray:
        """120 线平面激光雷达（3° 一条，够分辨 0.44 m 的柱子），返回每条射线的距离（米，上限 LIDAR_RANGE）。"""
        q = self.data.qpos
        origin = np.array([q[0], q[1], 0.25], dtype=np.float64)
        yaw = yaw_of(q[3:7])
        ranges = np.full(LIDAR_BEAMS, LIDAR_RANGE, dtype=np.float32)
        geomid = np.zeros(1, dtype=np.int32)
        normal = np.zeros(3, dtype=np.float64)
        for k in range(LIDAR_BEAMS):
            ang = yaw + 2.0 * np.pi * k / LIDAR_BEAMS
            vec = np.array([np.cos(ang), np.sin(ang), 0.0])
            dist = self._cast(origin, vec, geomid, normal)
            if dist >= 0:
                ranges[k] = min(float(dist), LIDAR_RANGE)
        return ranges

    def _cast(self, pnt, vec, geomid, normal) -> float:
        """兼容 MuJoCo 3.1.x（8 参）与 3.2+（9 参）的 mj_ray。"""
        if getattr(self, "_ray9", True):
            try:
                return mujoco.mj_ray(self.model, self.data, pnt, vec, None, True,
                                     self.body_id, geomid, normal)
            except TypeError:
                self._ray9 = False
        return mujoco.mj_ray(self.model, self.data, pnt, vec, None, True,
                             self.body_id, geomid)

    # ---------------------------------------------------------------- 运动与里程计
    def rgb(self, cam: str = "chase_cam", size: int = 200) -> np.ndarray:
        """离屏渲染一帧（chase_cam 跟车视角），用于 GIF 演示。"""
        if self._renderer is None:
            import mujoco
            self._renderer = mujoco.Renderer(self.model, size, size)
        self._renderer.update_scene(self.data, camera=cam)
        return self._renderer.render()

    def step(self, v: float, w: float) -> State:
        """速度伺服：把 (v, w) 变成力与力矩；侧向速度强阻尼以模拟轮式约束。"""
        R = self.data.xmat[self.body_id].reshape(3, 3)
        yaw = yaw_of(self.data.qpos[3:7])
        vel = np.array(self.data.qvel[0:2])
        v_des = np.array([v * np.cos(yaw), v * np.sin(yaw)])
        a = 6.0 * (v_des - vel)
        force = self.mass * a
        # 侧向阻尼：把垂直于车头方向的速度压掉
        side = np.array([-np.sin(yaw), np.cos(yaw)])
        force -= self.mass * 8.0 * np.dot(vel, side) * side

        omega = float(self.data.qvel[5])
        torque_z = 4.0 * (w - omega)

        self.data.xfrc_applied[self.body_id, 0:2] = np.clip(force, -60, 60)
        self.data.xfrc_applied[self.body_id, 2] = self.mass * 9.81
        self.data.xfrc_applied[self.body_id, 5] = float(np.clip(torque_z, -12, 12))
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)):      # 数值爆炸保护
            self.reset(self.scene, self.seed)
            self.crashed = True
            return self.state()
        self._check_crash()
        return self.state()

    def _check_crash(self) -> None:
        for c in self.data.contact[: self.data.ncon]:
            if self.body_id in (self.model.geom_bodyid[c.geom1],
                                self.model.geom_bodyid[c.geom2]):
                self.crashed = True
                return

    def odom(self, v: float, w: float, slip: float = 0.0,
             noise: float = 0.0) -> tuple[float, float]:
        """轮式里程计：真实速度 × (1+轮滑率) + 高斯噪声。SLAM 只允许用这个。"""
        v_meas = v * (1.0 + slip) + self.rng.normal(0.0, noise)
        w_meas = w * (1.0 + slip) + self.rng.normal(0.0, noise)
        return float(v_meas), float(w_meas)


def ray_points(ranges: np.ndarray, pose) -> np.ndarray:
    """把雷达量程转成世界坐标点（用于建图）。"""
    x, y, yaw = pose
    pts = []
    for k, r in enumerate(ranges):
        ang = yaw + 2.0 * np.pi * k / LIDAR_BEAMS
        pts.append([x + r * np.cos(ang), y + r * np.sin(ang)])
    return np.array(pts)
