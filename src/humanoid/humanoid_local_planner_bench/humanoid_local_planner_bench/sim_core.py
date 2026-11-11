# -*- coding: utf-8 -*-
"""场景与几何：位姿写入、激光扫描、碰撞与间距查询。

机器人做**运动学驱动**：每个控制周期把 (x, y, yaw) 直接写进 freejoint，
再 mj_forward。碰撞与间距都由 MuJoCo 用**真实几何体**计算，不做质点假设。
"""

import math
import os

import numpy as np

import humanoid_local_planner_bench.config as config

MODEL_NAME = "corridor_scene.xml"

# clearance() 依赖 mj_geomDistance 在「圆柱 vs box」上的结果。
# 3.2.x 上同一组几何体会返回 0.6873（正确值 1.2600），负值会让最小间距失去意义，
# 而 collided() 仍为 0，看上去像"没碰撞但间距为负"，排查成本很高。所以直接拦住。
MIN_MUJOCO = (3, 3, 0)


def _version_tuple(text):
    out = []
    for part in str(text).split(".")[:3]:
        num = ""
        for ch in part:
            if not ch.isdigit():
                break
            num += ch
        out.append(int(num) if num else 0)
    while len(out) < 3:
        out.append(0)
    return tuple(out)


def check_mujoco_version(mujoco):
    """版本低于 3.3 时抛错，避免 clearance() 静默算错。"""
    have = _version_tuple(getattr(mujoco, "__version__", "0"))
    if have < MIN_MUJOCO:
        raise RuntimeError(
            "MuJoCo %s 过低，需要 >= %d.%d.%d。3.2.x 的 mj_geomDistance 在"
            "「圆柱 vs box」上返回的间距偏小，clearance() 会出现负值。"
            " 升级：python3 -m pip install -U 'mujoco>=3.3'"
            % (getattr(mujoco, "__version__", "?"), MIN_MUJOCO[0], MIN_MUJOCO[1], MIN_MUJOCO[2]))

# 障碍物几何名（用于碰撞与间距判定，不含地板）
OBSTACLE_GEOMS = (
    "wall_south_a", "wall_south_b", "wall_north_a", "wall_north_b",
    "door_jamb_s", "door_jamb_n", "obs_a", "obs_b",
)


def default_model_path():
    """源码树 → realpath → ament share 三级回退。"""
    here = os.path.dirname(os.path.abspath(__file__))
    real = os.path.dirname(os.path.realpath(__file__))
    cands = [
        os.path.join(real, "..", "models", MODEL_NAME),
        os.path.join(here, "..", "models", MODEL_NAME),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(
            get_package_share_directory("humanoid_local_planner_bench"), "models", MODEL_NAME))
    except Exception:                                            # noqa: BLE001
        pass
    for p in cands:
        if os.path.isfile(p):
            return p
    return cands[0]


class CorridorSim:
    """走廊场景封装：位姿、激光、碰撞、间距。"""

    def __init__(self, model_path=None, n_rays=None, ray_range=None):
        import mujoco
        check_mujoco_version(mujoco)
        self.mujoco = mujoco
        with open(model_path or default_model_path(), encoding="utf-8") as f:
            xml = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.n_rays = int(n_rays or config.N_RAYS)
        self.ray_range = float(ray_range or config.RAY_RANGE)
        # 射线只考虑 group 0（墙/门柱/障碍），屏蔽 group 1 的机器人自身几何体
        self.ray_geomgroup = np.array([1, 0, 1, 1, 1, 1], dtype=np.uint8)
        self.footprint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "footprint")
        self.obstacles = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, n)
                          for n in OBSTACLE_GEOMS]
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot")
        self.ray_bodyexclude = self.body
        self.reset()

    # ------------------------------------------------------------------
    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.set_pose(x, y, yaw)
        self.collision_count = 0

    def set_pose(self, x, y, yaw, phase=0.0):
        """把机器人摆到指定位姿（运动学驱动），并按步态相位摆动腿部。

        phase 由行进距离累加得到（见 main 的 run 循环），步幅约 0.7 m。
        腿部只参与渲染（contype=0），不参与碰撞判定，因此步态不会影响指标。
        """
        mujoco = self.mujoco
        half = yaw * 0.5
        self.data.qpos[0] = x
        self.data.qpos[1] = y
        self.data.qpos[2] = 0.012 * (0.5 - 0.5 * math.cos(2 * phase))
        self.data.qpos[3] = math.cos(half)      # w
        self.data.qpos[4] = 0.0
        self.data.qpos[5] = 0.0
        self.data.qpos[6] = math.sin(half)      # z
        # 关节顺序：hip_l, knee_l, ankle_l, hip_r, knee_r, ankle_r
        for i, ph in ((0, phase), (1, phase + math.pi)):
            hip = -0.45 * math.sin(ph)
            knee = 1.05 * max(0.0, math.sin(ph))
            ankle = 0.30 * math.sin(ph)
            self.data.qpos[7 + i * 3] = hip
            self.data.qpos[8 + i * 3] = knee
            self.data.qpos[9 + i * 3] = ankle
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def pose(self):
        q = self.data.qpos
        w, z = float(q[3]), float(q[6])
        yaw = 2.0 * math.atan2(z, w)
        return float(q[0]), float(q[1]), yaw

    # ------------------------------------------------------------------
    def scan(self):
        """二维激光：n_rays 束均匀分布，返回 [(angle, distance), ...]。

        直接调用 MuJoCo 的射线求交，打的是场景里的真实几何体。
        """
        mujoco = self.mujoco
        x, y, yaw = self.pose()
        origin = np.array([x, y, config.SCAN_HEIGHT])
        out = []
        for i in range(self.n_rays):
            ang = yaw + 2.0 * math.pi * i / self.n_rays
            vec = np.array([math.cos(ang), math.sin(ang), 0.0])
            geomid = np.zeros(1, dtype=np.int32)
            dist = mujoco.mj_ray(self.model, self.data, origin, vec,
                                 self.ray_geomgroup, 1, self.ray_bodyexclude, geomid)
            if dist < 0 or dist > self.ray_range:
                dist = self.ray_range
            out.append((ang, float(dist)))
        return out

    def clearance(self, max_dist=3.0):
        """足迹到最近障碍的距离（m）。用真实几何体距离，不是圆心距。"""
        mujoco = self.mujoco
        best = max_dist
        for g in self.obstacles:
            try:
                d = mujoco.mj_geomDistance(self.model, self.data, self.footprint, g,
                                           max_dist, None)
            except Exception:                                    # noqa: BLE001
                d = max_dist
            if d is not None:
                best = min(best, float(d))
        return best

    def collided(self):
        """当前位姿下足迹是否与障碍物相交（考虑穿透容差）。"""
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if self.footprint in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == self.footprint else c.geom1
                if other in self.obstacles and c.dist < 0.01:
                    return True
        return False
