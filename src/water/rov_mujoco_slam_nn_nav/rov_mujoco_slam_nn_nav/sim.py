"""ROV 仿真内核：物理步进 + 6-DOF 控制 + 传感器 + 渲染。"""
from __future__ import annotations
import math
from typing import Optional
import numpy as np
import mujoco
from . import model as M
from .control import ROVController, drag_force, mat_to_rpy, quat_to_mat


class ROVSim:
    def __init__(self, scene="pool", seed=0, layout=None, render=False):
        self.layout = layout or M.make_layout(scene, seed)
        self.scene = self.layout.name
        self.model = mujoco.MjModel.from_xml_string(M.build_xml(self.layout))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 0.002
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "rov")
        self.sonar_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "sonar")
        self.cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "fpv")
        total_mass = float(self.model.body_subtreemass[self.body_id])
        self.ctrl = ROVController(mass=total_mass)
        self.cmd = {"surge": 0.0, "sway": 0.0, "heave": 0.0,
                    "yaw_rate": 0.0, "depth": M.CRUISE_DEPTH}
        self.renderer = None
        self._render_size = None
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        sx, sy, syaw = self.layout.start
        self.data.qpos[0:3] = [sx, sy, M.CRUISE_DEPTH]
        self.data.qpos[3:7] = [math.cos(syaw/2), 0.0, 0.0, math.sin(syaw/2)]
        mujoco.mj_forward(self.model, self.data)
        self.ctrl.reset(syaw)
        self.cmd = {"surge": 0.0, "sway": 0.0, "heave": 0.0,
                    "yaw_rate": 0.0, "depth": M.CRUISE_DEPTH}
        for _ in range(250):
            self._physics_step()

    def set_command(self, surge=0.0, sway=0.0, heave=0.0, yaw_rate=0.0, depth=None):
        self.cmd["surge"] = float(np.clip(surge, -1.2, 1.2))
        self.cmd["sway"] = float(np.clip(sway, -1.0, 1.0))
        self.cmd["heave"] = float(np.clip(heave, -0.6, 0.6))
        self.cmd["yaw_rate"] = float(np.clip(yaw_rate, -1.0, 1.0))
        if depth is not None:
            self.cmd["depth"] = float(np.clip(depth, 0.4, 3.5))

    def _physics_step(self):
        force, torque = self.ctrl.compute(self.data.qpos[0:3], self.data.qpos[3:7],
                                          self.data.qvel[0:3], self.data.qvel[3:6],
                                          self.cmd, self.model.opt.timestep)
        if self.cmd["heave"]:
            force[2] += self.cmd["heave"] * 40.0
        force = np.array(force, dtype=float)
        force[2] += M.BUOYANCY          # 浮力
        force -= drag_force(self.data.qvel[0:3])   # 流体阻力
        self.data.xfrc_applied[self.body_id, 0:3] = force
        self.data.xfrc_applied[self.body_id, 3:6] = torque
        mujoco.mj_step(self.model, self.data)

    def step(self, n=1):
        for _ in range(n):
            self._physics_step()

    def step_dt(self, dt):
        n = max(1, int(round(dt / self.model.opt.timestep)))
        self.step(n)
        return n * self.model.opt.timestep

    def base_pos(self):
        return np.array(self.data.qpos[0:3])

    def base_quat(self):
        return np.array(self.data.qpos[3:7])

    def rpy(self):
        return mat_to_rpy(quat_to_mat(self.base_quat()))

    def base_yaw(self):
        return self.rpy()[2]

    def velocity(self):
        return np.array(self.data.qvel[0:3])

    def is_fallen(self):
        roll, pitch, _ = self.rpy()
        return bool(self.data.qpos[2] < 0.05 or abs(roll) > 1.2 or abs(pitch) > 1.2)

    def state(self):
        roll, pitch, yaw = self.rpy()
        return {"pos": self.base_pos().tolist(), "yaw": yaw, "roll": roll, "pitch": pitch,
                "vel": self.velocity().tolist(), "fallen": self.is_fallen()}

    def _ensure_renderer(self, w, h):
        if self.renderer is None or self._render_size != (w, h):
            from mujoco.rendering.classic.renderer import Renderer
            self.renderer = Renderer(self.model, h, w)
            try:
                self.renderer._scene_option.geomgroup[:] = 1
            except Exception:
                pass
            self._render_size = (w, h)
            mujoco.mj_forward(self.model, self.data)

    def follow_view(self, width=480, height=360, distance=3.0, azimuth=135.0, elevation=-20.0):
        st = self.state()
        self._ensure_renderer(width, height)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [st["pos"][0], st["pos"][1], st["pos"][2]]
        cam.distance = distance
        cam.azimuth = azimuth
        cam.elevation = elevation
        self.renderer.update_scene(self.data, camera=cam)
        return self.renderer.render()

    def render(self, width=480, height=360, camera=None):
        self._ensure_renderer(width, height)
        if camera is None:
            self.renderer.update_scene(self.data)
        else:
            cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
            self.renderer.update_scene(self.data, camera=cid)
        return self.renderer.render()
