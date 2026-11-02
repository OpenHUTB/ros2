"""sim.py —— 多模态跌倒预警仿真封装"""
from __future__ import annotations
import os
import numpy as np
try:
    import mujoco
except (ImportError, OSError, RuntimeError):
    mujoco = None

DEFAULT_STAND_TARGET = np.array([
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)


class FallWarningSim:
    def __init__(self, model_path, kp=60.0, kd=6.0):
        if mujoco is None:
            raise RuntimeError("mujoco 未安装")
        with open(model_path, "r", encoding="utf-8") as f:
            xml_str = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml_str)
        self.data = mujoco.MjData(self.model)
        self.kp, self.kd = kp, kd
        self.qpos_js, self.qvel_js = 7, 6
        self._renderer = None
        self._cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_js:self.qpos_js+15] = DEFAULT_STAND_TARGET
        mujoco.mj_forward(self.model, self.data)

    def step(self, target=None):
        if target is None: target = DEFAULT_STAND_TARGET
        qj = self.data.qpos[self.qpos_js:self.qpos_js+15]
        qv = self.data.qvel[self.qvel_js:self.qvel_js+15]
        tau = np.clip(self.kp*(target-qj)-self.kd*qv, -280.0, 280.0)
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)

    def apply_push(self, fxyz, duration_steps=40):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        for _ in range(max(1, int(duration_steps))):
            self.data.xfrc_applied[bid, :3] = list(fxyz)
            self.step()
        self.data.xfrc_applied[bid, :3] = [0.0, 0.0, 0.0]

    @property
    def root_z(self): return float(self.data.qpos[2])
    def fallen(self, z=0.6): return self.root_z < z

    def torso_pose(self):
        """返回躯干 (pitch, roll) 弧度，ZYX欧拉角顺序。"""
        qw, qx, qy, qz = self.data.qpos[3:7]
        # roll (绕x轴)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = float(np.arctan2(sinr_cosp, cosr_cosp))
        # pitch (绕y轴)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = float(np.arcsin(np.clip(sinp, -1, 1)))
        return pitch, roll

    def render_front(self, gray=True):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 64, 64)
        self._renderer.update_scene(self.data, camera=self._cam_id)
        img = self._renderer.render()
        if gray:
            g = 0.299*img[...,0]+0.587*img[...,1]+0.114*img[...,2]
            return g.astype(np.float32)/255.0
        return img

    def close(self):
        if self._renderer:
            try: self._renderer.close()
            except: pass
            self._renderer = None
