"""
sim.py —— MuJoCo 人形机器人仿真（端到端模块用）
==============================================
复用 humanoid_camera.xml 模型。提供站姿 PD、推力、前视相机渲染、躯干倾角读取。
"""
from __future__ import annotations
import os
import numpy as np
try:
    import mujoco
except ImportError:
    mujoco = None

JOINT_NAMES = [
    "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_pitch_l", "ankle_roll_l",
    "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_pitch_r", "ankle_roll_r",
    "shoulder_pitch_l", "elbow_l", "shoulder_pitch_r", "elbow_r", "neck",
]
DEFAULT_STAND_TARGET = np.array([
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)


class E2ESim:
    def __init__(self, model_path: str, kp: float = 60.0, kd: float = 6.0,
                 image_h: int = 64, image_w: int = 64):
        if mujoco is None:
            raise RuntimeError("mujoco 未安装")
        with open(model_path, "r", encoding="utf-8") as f:
            xml_str = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml_str)
        self.data = mujoco.MjData(self.model)
        self.kp, self.kd = float(kp), float(kd)
        self.image_h, self.image_w = image_h, image_w
        self.qpos_j0, self.qvel_j0 = 7, 6
        self._renderer = None
        self._cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_j0:self.qpos_j0+15] = DEFAULT_STAND_TARGET
        mujoco.mj_forward(self.model, self.data)

    def step(self, target: np.ndarray | None = None):
        if target is None:
            target = DEFAULT_STAND_TARGET
        qj = self.data.qpos[self.qpos_j0:self.qpos_j0+15]
        qv = self.data.qvel[self.qvel_j0:self.qvel_j0+15]
        tau = np.clip(self.kp * (target - qj) - self.kd * qv, -280, 280)
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)

    def apply_push(self, force_xyz, steps=40):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        for _ in range(max(1, int(steps))):
            self.data.xfrc_applied[bid, :3] = list(force_xyz)
            self.step()
        self.data.xfrc_applied[bid, :3] = [0, 0, 0]

    @property
    def root_z(self):
        return float(self.data.qpos[2])

    def fallen(self, z_th=0.6):
        return self.root_z < z_th

    def torso_tilt(self):
        qw, qx, qy, qz = self.data.qpos[3:7]
        fz = 2*(qx*qz + qw*qy)
        fx = 1 - 2*(qy*qy + qz*qz)
        return float(np.arctan2(fz, fx))

    def torso_tilt_y(self):
        qw, qx, qy, qz = self.data.qpos[3:7]
        fy = 2*(qx*qy - qw*qz)
        fz = 1 - 2*(qx*qx + qz*qz)
        return float(np.arctan2(fy, fz))

    def render_front(self, gray=True):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.image_h, self.image_w)
        self._renderer.update_scene(self.data, camera=self._cam_id)
        img = self._renderer.render()
        if gray:
            return (0.299*img[...,0] + 0.587*img[...,1] + 0.114*img[...,2]).astype(np.float32)/255.0
        return img

    def close(self):
        if self._renderer:
            try: self._renderer.close()
            except Exception: pass
            self._renderer = None
