# -*- coding: utf-8 -*-
"""场景封装：位姿写入、足端世界坐标、脚与台阶的真实几何距离。

机器人做**运动学驱动**：每个控制周期把 (base_x, base_z) 和 6 个关节角直接写进
qpos，再 mj_forward。台阶与脚都是真实几何体，脚底到台阶面的距离用
mj_geomDistance 计算，不用中心点近似。
"""

import math
import os

import numpy as np

import humanoid_stair_climb.config as config

MODEL_NAME = config.MODEL_NAME

# 台阶类几何（含平台），用于量脚底距离。必须覆盖全部台阶，
# 少一级就可能漏掉那一段的穿模。
STEP_GEOMS = tuple("step_%d" % i for i in range(1, config.STEP_COUNT + 1)) + ("landing",)

# qpos 布局：0-2 base 位置，3-6 base 四元数，7-12 六个关节
QPOS_FREE = 7
JOINT_ORDER = ("hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r")


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
            get_package_share_directory(config.PACKAGE_NAME), "models", MODEL_NAME))
    except Exception:                                            # noqa: BLE001
        pass
    for p in cands:
        if os.path.isfile(p):
            return p
    return cands[0]


class StairSim:
    """楼梯场景：写位姿、读足端、量间距。"""

    def __init__(self, model_path=None):
        import mujoco
        self.mujoco = mujoco
        with open(model_path or default_model_path(), encoding="utf-8") as f:
            xml = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)

        self.robot = self._id(mujoco.mjtObj.mjOBJ_BODY, "robot")
        self.foot_geom = {
            "l": self._id(mujoco.mjtObj.mjOBJ_GEOM, "foot_l"),
            "r": self._id(mujoco.mjtObj.mjOBJ_GEOM, "foot_r"),
        }
        self.step_geoms = [self._id(mujoco.mjtObj.mjOBJ_GEOM, n) for n in STEP_GEOMS]
        self.reset()

    def _id(self, obj, name):
        idx = self.mujoco.mj_name2id(self.model, obj, name)
        if idx < 0:
            raise RuntimeError("模型里找不到 %s" % name)
        return idx

    # ------------------------------------------------------------------
    def reset(self, base_x=None, base_z=None, joints=None):
        self.set_state(
            config.BASE_X0 if base_x is None else base_x,
            config.STAND_Z if base_z is None else base_z,
            joints if joints is not None else [0.0] * 6,
        )

    def set_state(self, base_x, base_z, joints):
        """写入基座平面位置与 6 个关节角（yaw 恒为 0，沿 +x 前进）。"""
        q = self.data.qpos
        q[0] = base_x
        q[1] = 0.0
        q[2] = base_z
        q[3], q[4], q[5], q[6] = 1.0, 0.0, 0.0, 0.0     # w, x, y, z
        for i, ang in enumerate(joints):
            q[QPOS_FREE + i] = ang
        self.data.qvel[:] = 0.0
        self.mujoco.mj_forward(self.model, self.data)

    def base_pos(self):
        p = self.data.qpos
        return float(p[0]), float(p[1]), float(p[2])

    def foot_center(self, side):
        """脚几何中心的世界坐标。"""
        return np.array(self.data.geom_xpos[self.foot_geom[side]], dtype=float)

    def foot_bottom(self, side):
        """脚底中心的世界坐标（脚是 0.04 m 厚的盒子，中心下移 0.02）。"""
        return self.foot_center(side) - np.array([0.0, 0.0, 0.02])

    def foot_clearance(self, side, max_dist=0.60):
        """脚底几何到最近台阶面的距离（负值表示穿透）。"""
        best = max_dist
        for g in self.step_geoms:
            try:
                d = self.mujoco.mj_geomDistance(
                    self.model, self.data, self.foot_geom[side], g, max_dist, None)
            except Exception:                                    # noqa: BLE001
                continue
            if d is not None:
                best = min(best, float(d))
        # 平地部分：脚底到 z=0 的竖直距离
        bottom = self.foot_bottom(side)
        if bottom[0] < 0.0:
            best = min(best, float(bottom[2]))
        return best

    def clearance(self):
        """两只脚里离障碍更近的那个距离。"""
        return min(self.foot_clearance("l"), self.foot_clearance("r"))

    def reached(self):
        """是否已站上平台（x 到位且基座升到位）。"""
        x, _, z = self.base_pos()
        return x >= config.REACH_X and z >= config.REACH_Z


def yaw_to_quat(yaw):
    """仅用于外部工具：yaw → (w, x, y, z)。"""
    return math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)
