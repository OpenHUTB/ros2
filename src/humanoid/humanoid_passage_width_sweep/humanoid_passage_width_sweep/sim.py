# -*- coding: utf-8 -*-
"""MuJoCo 仿真封装：激光扫描、运动学推进、碰撞检测、遥测采集、离屏渲染。

人形在这里是**运动学驱动**的（直接积分位姿，不受力），因为本模块评价的是
规划层，不是平衡控制；碰撞体是半径 0.30 m 的圆柱，用于给出权威的碰撞判定。
"""

from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

import humanoid_passage_width_sweep.config as config
from humanoid_passage_width_sweep import planners, scene

_PKG_DIR = Path(__file__).resolve().parent
_MODEL_CANDIDATES = [
    _PKG_DIR.parent / "models" / config.MODEL_NAME,
    _PKG_DIR / "models" / config.MODEL_NAME,
]


def find_model() -> Path:
    """定位随包安装的 corridor_scene.xml（--gui 与一致性测试用）。"""
    for p in _MODEL_CANDIDATES:
        if p.is_file():
            return p
    tried = "\n".join(f"  - {p}" for p in _MODEL_CANDIDATES)
    raise FileNotFoundError(f"找不到 {config.MODEL_NAME}，已尝试：\n{tried}")


class PassageSim:
    """窄通道通行仿真。

    :param width: 窄门净宽 (m)
    :param variant: 变体名（见 planners.VARIANTS）
    :param add_obstacles: 是否放圆柱障碍
    """

    def __init__(self, width: float = config.DEFAULT_WIDTH,
                 variant: str = "mppi_footprint", add_obstacles: bool = True):
        import mujoco

        self._mj = mujoco
        self.width = float(width)
        self.xml = scene.build_scene_xml(self.width, add_obstacles=add_obstacles)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = config.TIMESTEP

        m = self.model
        self._jx = int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "root_x")])
        self._jy = int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "root_y")])
        self._jyaw = int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "root_yaw")])
        self._footprint = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "footprint"))
        self._floor = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor"))
        self._torso_body = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso"))

        self.planner = planners.make(variant)
        self.variant = variant
        self.planner_name = self.planner.name
        self._rays = np.linspace(-np.pi, np.pi, config.N_RAYS, endpoint=False)
        self.renderer = None
        self.time = 0.0
        self.reset()

    # ------------------------------------------------------------------ 状态
    def reset(self) -> None:
        self._mj.mj_resetData(self.model, self.data)
        d = self.data
        d.qpos[self._jx] = config.START[0]
        d.qpos[self._jy] = config.START[1]
        d.qpos[self._jyaw] = config.START[2]
        self.planner.reset()
        self._mj.mj_forward(self.model, d)
        self.time = 0.0

    def pose(self):
        """当前位姿 (x, y, yaw)。"""
        d = self.data
        return float(d.qpos[self._jx]), float(d.qpos[self._jy]), float(d.qpos[self._jyaw])

    def in_contact(self) -> bool:
        """足迹是否与**墙/障碍**接触（权威碰撞判定）。

        必须排除地面：足迹圆柱高 1.7 m、底面贴地，否则每一帧都会被判成碰撞
        （早期版本因此让所有实验在 0.1 s 就终止）。
        """
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            a, b = int(c.geom1), int(c.geom2)
            if self._footprint not in (a, b):
                continue
            other = b if a == self._footprint else a
            if other == self._floor:
                continue
            return True
        return False

    # ------------------------------------------------------------------ 激光
    def scan(self) -> np.ndarray:
        """64 束激光，返回世界坐标命中点 (M, 2)，未命中的行是 NaN。"""
        m, d = self.model, self.data
        x, y, yaw = self.pose()
        pnt = np.array([x, y, config.RAY_Z])
        out = np.full((config.N_RAYS, 2), np.nan)
        geomid = np.zeros(1, dtype=np.int32)
        for i, a in enumerate(self._rays):
            ang = yaw + a
            vec = np.array([np.cos(ang), np.sin(ang), 0.0])
            dist = self._mj.mj_ray(m, d, pnt, vec, None, 1, self._torso_body, geomid)
            if dist >= 0.0 and dist <= config.RAY_RANGE:
                out[i] = [x + vec[0] * dist, y + vec[1] * dist]
        return out

    def clearance(self, scan_points: Optional[np.ndarray] = None) -> float:
        """足迹边缘到最近障碍的距离；无命中返回 ``RAY_RANGE - r``。"""
        pts = self.scan() if scan_points is None else scan_points
        if pts.size == 0 or not np.isfinite(pts).any():
            return config.RAY_RANGE - config.FOOTPRINT_RADIUS
        d = np.nanmin(np.linalg.norm(pts - np.array(self.pose()[:2]), axis=1))
        return float(d - config.FOOTPRINT_RADIUS)

    # ------------------------------------------------------------------ 推进
    def step(self) -> Dict[str, float]:
        """推进一个控制步（= ``1/CONTROL_HZ`` 秒 = 5 个物理步）。"""
        x, y, yaw = self.pose()
        pts = self.scan()
        v, w = self.planner.compute(x, y, yaw, pts)
        n_sub = max(1, int(round(1.0 / config.CONTROL_HZ / config.TIMESTEP)))
        dt = 1.0 / config.CONTROL_HZ
        yaw_mid = yaw + 0.5 * w * dt
        self.data.qpos[self._jx] = x + v * np.cos(yaw_mid) * dt
        self.data.qpos[self._jy] = y + v * np.sin(yaw_mid) * dt
        self.data.qpos[self._jyaw] = float(np.clip(yaw + w * dt, -np.pi, np.pi))
        self._mj.mj_forward(self.model, self.data)
        self.time += dt
        return dict(t=self.time, x=self.data.qpos[self._jx], y=self.data.qpos[self._jy],
                    yaw=self.data.qpos[self._jyaw], v=v, w=w,
                    clearance=self.clearance(pts), contact=float(self.in_contact()))

    def goal_distance(self) -> float:
        x, y, _ = self.pose()
        return float(np.hypot(config.GOAL[0] - x, config.GOAL[1] - y))

    def run(self, duration: float = config.TIME_LIMIT) -> Dict[str, np.ndarray]:
        """跑到「到达目标 / 碰撞 / 超时」三者之一，返回等长遥测。

        **碰撞即终止**：人形是运动学驱动的，MuJoCo 不会挡住它，若不主动终止，
        规划器会「穿墙」到达终点，把失败伪装成成功（早期版本就撞过这个坑：
        mppi 在 0.70 m 门口报告 reached=True 但 contact 了 502 步）。
        """
        keys = ("t", "x", "y", "yaw", "v", "w", "clearance", "contact")
        log: Dict[str, List[float]] = {k: [] for k in keys}
        n = int(round(duration * config.CONTROL_HZ))
        self.failed = False
        for _ in range(n):
            s = self.step()
            for k in keys:
                log[k].append(s[k])
            if s["contact"] > 0.5:
                self.failed = True
                break
            if self.goal_distance() <= config.GOAL_RADIUS:
                break
        return {k: np.asarray(v, dtype=float) for k, v in log.items()}

    # ------------------------------------------------------------------ 渲染
    def render(self, width: int = 640, height: int = 360, camera: str = "overview") -> np.ndarray:
        mj = self._mj
        if self.renderer is None:
            self.renderer = mj.Renderer(self.model, height, width)
        cam = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_CAMERA, camera)
        self.renderer.update_scene(self.data, camera=cam if cam >= 0 else None)
        return self.renderer.render()

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
