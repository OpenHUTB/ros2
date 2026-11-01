"""四旋翼仿真内核：物理步进 + 飞行控制 + 传感器 + 渲染。"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

import mujoco

from . import model as M
from .flight import FlightController, mat_to_rpy, quat_to_mat


class DroneSim:
    def __init__(self, scene: str = "warehouse", seed: int = 0,
                 layout: Optional[M.SceneLayout] = None, render: bool = False):
        self.layout = layout or M.make_layout(scene, seed)
        self.scene = self.layout.name
        self.model = mujoco.MjModel.from_xml_string(M.build_xml(self.layout))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 0.002
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "drone")
        self.lidar_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
        self.cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "fpv")
        total_mass = float(self.model.body_subtreemass[self.body_id])
        self.ctrl = FlightController(mass=total_mass)
        self.cmd = {"vx": 0.0, "vy": 0.0, "yaw_rate": 0.0, "z_ref": M.FLIGHT_ALT}
        self.renderer = None
        self._render_size = None
        self._want_render = bool(render)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        sx, sy, syaw = self.layout.start
        self.data.qpos[0:3] = [sx, sy, M.FLIGHT_ALT]
        self.data.qpos[3:7] = [math.cos(syaw / 2), 0.0, 0.0, math.sin(syaw / 2)]
        mujoco.mj_forward(self.model, self.data)
        self.ctrl.reset(syaw)
        self.cmd = {"vx": 0.0, "vy": 0.0, "yaw_rate": 0.0, "z_ref": M.FLIGHT_ALT}
        for _ in range(300):        # 悬停 0.6 s 稳定
            self._physics_step()

    def set_command(self, vx: float = 0.0, vy: float = 0.0, yaw_rate: float = 0.0,
                    z_ref: Optional[float] = None):
        self.cmd["vx"] = float(np.clip(vx, -1.5, 1.5))
        self.cmd["vy"] = float(np.clip(vy, -1.5, 1.5))
        self.cmd["yaw_rate"] = float(np.clip(yaw_rate, -1.2, 1.2))
        if z_ref is not None:
            self.cmd["z_ref"] = float(np.clip(z_ref, 0.6, 2.2))

    # ------------------------------------------------------------------
    def _physics_step(self):
        pos = self.data.qpos[0:3]
        quat = self.data.qpos[3:7]
        vel = self.data.qvel[0:3]
        omega = self.data.qvel[3:6]
        force, torque, _ = self.ctrl.compute(pos, quat, vel, omega,
                                             self.cmd["vx"], self.cmd["vy"],
                                             self.cmd["yaw_rate"], self.cmd["z_ref"])
        self.data.xfrc_applied[self.body_id, 0:3] = force
        self.data.xfrc_applied[self.body_id, 3:6] = torque
        mujoco.mj_step(self.model, self.data)

    def step(self, n: int = 1):
        for _ in range(n):
            self._physics_step()

    def step_dt(self, dt: float):
        n = max(1, int(round(dt / self.model.opt.timestep)))
        self.step(n)
        return n * self.model.opt.timestep

    # ------------------------------------------------------------------
    def base_pos(self) -> np.ndarray:
        return np.array(self.data.qpos[0:3])

    def base_quat(self) -> np.ndarray:
        return np.array(self.data.qpos[3:7])

    def rpy(self):
        return mat_to_rpy(quat_to_mat(self.base_quat()))

    def base_yaw(self) -> float:
        return self.rpy()[2]

    def velocity(self) -> np.ndarray:
        return np.array(self.data.qvel[0:3])

    def body_velocity(self) -> np.ndarray:
        r = quat_to_mat(self.base_quat())
        return r.T @ self.velocity()

    def is_fallen(self) -> bool:
        roll, pitch, _ = self.rpy()
        return bool(self.data.qpos[2] < 0.3 or abs(roll) > 1.2 or abs(pitch) > 1.2)

    def state(self) -> dict:
        roll, pitch, yaw = self.rpy()
        return {
            "pos": self.base_pos().tolist(),
            "yaw": yaw, "roll": roll, "pitch": pitch,
            "vel": self.velocity().tolist(),
            "fallen": self.is_fallen(),
        }

    # ------------------------------------------------------------------
    def render(self, width: int = 480, height: int = 360, camera: Optional[str] = None):
        if self.renderer is None or self._render_size != (width, height):
            from mujoco.rendering.classic.renderer import Renderer
            self.renderer = Renderer(self.model, height, width)
            try:
                self.renderer._scene_option.geomgroup[:] = 1
            except Exception:
                pass
            self._render_size = (width, height)
            mujoco.mj_forward(self.model, self.data)
        if camera is None:
            self.renderer.update_scene(self.data)
        else:
            cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
            self.renderer.update_scene(self.data, camera=cid)
        return self.renderer.render()

    def follow_view(self, width: int = 480, height: int = 360, distance: float = 3.2,
                    azimuth: float = 135.0, elevation: float = -22.0):
        st = self.state()
        if self.renderer is None or self._render_size != (width, height):
            from mujoco.rendering.classic.renderer import Renderer
            self.renderer = Renderer(self.model, height, width)
            try:
                self.renderer._scene_option.geomgroup[:] = 1
            except Exception:
                pass
            self._render_size = (width, height)
            mujoco.mj_forward(self.model, self.data)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [st["pos"][0], st["pos"][1], st["pos"][2]]
        cam.distance = distance
        cam.azimuth = azimuth
        cam.elevation = elevation
        self.renderer.update_scene(self.data, camera=cam)
        return self.renderer.render()
