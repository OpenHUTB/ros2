"""MuJoCo 仿真内核。

对外只暴露与机器人无关的接口：``reset()`` / ``set_targets()`` / ``step()``，
ROS 节点与离线训练脚本共用同一份实现，保证"仿真里调通的控制器"能直接进 ROS。
"""

from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np

import mujoco

from . import model as M


def _quat_to_rpy(q) -> tuple:
    """MuJoCo (w,x,y,z) -> 欧拉角 (roll, pitch, yaw)，ZYX 顺序。"""
    w, x, y, z = q
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class HumanoidSim:
    """一个场景 + 一个人形机器人的仿真实例。"""

    def __init__(self, scene: str = "warehouse", seed: int = 0,
                 layout: Optional[M.SceneLayout] = None, render: bool = False):
        self.layout = layout or M.make_layout(scene, seed)
        self.scene = self.layout.name
        self.seed = seed
        self.xml = M.build_xml(self.layout)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 0.002

        self.root_qadr = self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")]
        self.joint_names = list(M.ACTUATED_JOINTS)
        self.qadr = {}
        self.dadr = {}
        for name in self.joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.qadr[name] = self.model.jnt_qposadr[jid]
            self.dadr[name] = self.model.jnt_dofadr[jid]
        self.act_id = {}
        for name in self.joint_names:
            self.act_id[name] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "a_" + name)
        self.cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "head_cam")
        self.goal = np.array(self.layout.goal, dtype=float)

        self.renderer = None
        self._want_render = bool(render)

        self.reset()

    # ------------------------------------------------------------------
    # 基础
    # ------------------------------------------------------------------
    def _init_renderer(self, width: int = 640, height: int = 480):
        try:
            from mujoco.rendering.classic.renderer import Renderer  # mujoco >= 3.15
        except Exception:  # pragma: no cover - 老版本
            Renderer = getattr(mujoco, "Renderer", None)
        if Renderer is None:
            raise RuntimeError("当前 MuJoCo 版本不含离屏渲染")
        self.renderer = Renderer(self.model, height, width)
        # 机器人几何放在 group 3（供雷达过滤），渲染时显式打开
        try:
            self.renderer._scene_option.geomgroup[:] = 1
        except Exception:
            pass
        self._render_size = (width, height)

    def reset(self, base: Optional[np.ndarray] = None, yaw: Optional[float] = None):
        from .kinematics import default_stand

        mujoco.mj_resetData(self.model, self.data)
        sx, sy, syaw = self.layout.start
        pelvis, q = default_stand((base[0], base[1]) if base is not None else (sx, sy),
                                  syaw if yaw is None else yaw)
        if base is not None:
            pelvis = np.array([base[0], base[1], base[2]])
        self.data.qpos[0:3] = pelvis
        cy, syaw = math.cos((syaw if yaw is None else yaw) / 2), math.sin(
            (syaw if yaw is None else yaw) / 2)
        self.data.qpos[3:7] = (cy, 0.0, 0.0, syaw)
        for name, val in q.items():
            self.data.qpos[self.qadr[name]] = val
        mujoco.mj_forward(self.model, self.data)
        self.set_targets(q)
        self._target = dict(q)
        return self.state()

    def set_targets(self, q: Dict[str, float]):
        for name, val in q.items():
            aid = self.act_id.get(name)
            if aid is not None and aid >= 0:
                self.data.ctrl[aid] = float(val)
        self._target = dict(q)

    def set_velocity(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
        """直接给根部速度（键盘实测模式下的低层接口，用于对比运动学控制）。"""
        yaw = self.base_yaw()
        self.data.qvel[0] = vx * math.cos(yaw) - vy * math.sin(yaw)
        self.data.qvel[1] = vx * math.sin(yaw) + vy * math.cos(yaw)
        self.data.qvel[5] = wz

    def step(self, n: int = 1, control=None):
        for _ in range(n):
            if control is not None:
                self.set_targets(control(self))
            mujoco.mj_step(self.model, self.data)

    def step_dt(self, dt: float):
        n = max(1, int(round(dt / self.model.opt.timestep)))
        muc = self.model.opt.timestep
        self.step(n)
        return n * muc

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------
    def base_pos(self) -> np.ndarray:
        return np.array(self.data.qpos[0:3])

    def base_quat(self) -> np.ndarray:
        return np.array(self.data.qpos[3:7])

    def base_yaw(self) -> float:
        return _quat_to_rpy(self.base_quat())[2]

    def base_rpy(self) -> tuple:
        return _quat_to_rpy(self.base_quat())

    def joint_positions(self) -> Dict[str, float]:
        return {n: float(self.data.qpos[self.qadr[n]]) for n in self.joint_names}

    def joint_velocities(self) -> Dict[str, float]:
        return {n: float(self.data.qvel[self.dadr[n]]) for n in self.joint_names}

    def ankle_pos(self, side: str) -> np.ndarray:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_ankle")
        return np.array(self.data.site_xpos[sid])

    def pelvis_velocity(self) -> np.ndarray:
        return np.array(self.data.qvel[0:3])

    def ground_height(self) -> float:
        return 0.0

    def is_fallen(self) -> bool:
        return float(self.data.qpos[2]) < 0.45

    def foot_contacts(self) -> tuple:
        """返回 (左脚触地, 右脚触地)。"""
        left = right = False
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            n1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, con.geom1) or ""
            n2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, con.geom2) or ""
            if "l_sole" in (n1, n2):
                left = True
            if "r_sole" in (n1, n2):
                right = True
        return left, right

    def state(self) -> dict:
        roll, pitch, yaw = self.base_rpy()
        return {
            "pos": self.base_pos().tolist(),
            "yaw": yaw,
            "roll": roll,
            "pitch": pitch,
            "vel": self.pelvis_velocity().tolist(),
            "joints": self.joint_positions(),
            "fallen": self.is_fallen(),
        }

    # ------------------------------------------------------------------
    # 渲染
    # ------------------------------------------------------------------
    def render(self, width: int = 640, height: int = 480, camera: Optional[str] = None,
               depth: bool = False):
        if self.renderer is None or self._render_size != (width, height):
            self._init_renderer(width, height)
            mujoco.mj_forward(self.model, self.data)
        if camera is None:
            self.renderer.update_scene(self.data)
        else:
            cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
            self.renderer.update_scene(self.data, camera=cid)
        return self.renderer.render()

    def camera_image(self, width: int = 160, height: int = 120) -> np.ndarray:
        return self.render(width, height, camera="head_cam")

    def follow_view(self, width: int = 480, height: int = 360, distance: float = 3.4,
                    azimuth: float = 140.0, elevation: float = -18.0):
        """第三人称跟随视角（自由相机 lookat 机器人）。"""
        st = self.state()
        if self.renderer is None or self._render_size != (width, height):
            self._init_renderer(width, height)
            mujoco.mj_forward(self.model, self.data)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [st["pos"][0], st["pos"][1], 0.55]
        cam.distance = distance
        cam.azimuth = azimuth
        cam.elevation = elevation
        self.renderer.update_scene(self.data, camera=cam)
        return self.renderer.render()

    def reset_to(self, x: float, y: float, yaw: float):
        return self.reset(base=np.array([x, y]), yaw=yaw)
