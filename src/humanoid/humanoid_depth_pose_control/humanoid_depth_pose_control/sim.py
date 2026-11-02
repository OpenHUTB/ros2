"""sim.py —— 深度图姿态估计用仿真封装"""
from __future__ import annotations
import os
import numpy as np

try:
    import mujoco
except (ImportError, OSError, RuntimeError):
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


class DepthPoseSim:
    def __init__(self, model_path, kp=60.0, kd=6.0, image_h=64, image_w=64):
        if mujoco is None:
            raise RuntimeError("mujoco 未安装")
        with open(model_path, "r", encoding="utf-8") as f:
            xml_str = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml_str)
        self.data = mujoco.MjData(self.model)
        self.kp, self.kd = kp, kd
        self.image_h, self.image_w = image_h, image_w
        self.qpos_js, self.qvel_js = 7, 6
        self._renderer = None
        self._cam_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_js:self.qpos_js+15] = DEFAULT_STAND_TARGET
        mujoco.mj_forward(self.model, self.data)

    def step(self, target=None):
        if target is None:
            target = DEFAULT_STAND_TARGET
        qj = self.data.qpos[self.qpos_js:self.qpos_js+15]
        qv = self.data.qvel[self.qvel_js:self.qvel_js+15]
        tau = np.clip(self.kp*(target-qj) - self.kd*qv, -280.0, 280.0)
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)

    def apply_push(self, fxyz, duration_steps=40):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        fx, fy, fz = fxyz
        for _ in range(max(1, int(duration_steps))):
            self.data.xfrc_applied[bid, :3] = [fx, fy, fz]
            self.step()
        self.data.xfrc_applied[bid, :3] = [0.0, 0.0, 0.0]

    @property
    def root_z(self):
        return float(self.data.qpos[2])

    def fallen(self, z_thresh=0.6):
        return self.root_z < z_thresh

    def torso_pose(self):
        """返回躯干 (pitch, roll, yaw) 弧度，ZYX欧拉角顺序。"""
        qw, qx, qy, qz = self.data.qpos[3:7]
        # roll (绕x轴)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = float(np.arctan2(sinr_cosp, cosr_cosp))
        # pitch (绕y轴)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = float(np.arcsin(np.clip(sinp, -1, 1)))
        # yaw (绕z轴)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        return pitch, roll, yaw

    def render_depth(self):
        """渲染深度图，返回 (H,W) float32，归一化到[0,1]。

        使用MuJoCo离屏渲染的真实深度缓冲，而非RGB亮度近似。
        """
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.image_h, self.image_w)
        self._renderer.update_scene(self.data, camera=self._cam_id)
        # MuJoCo 2.3+ 支持 render(depth=True) 获取真实深度缓冲
        try:
            depth = self._renderer.render(depth=True)
            # 深度值为距离(m)，归一化到[0,1]（近处=0，远处=1）
            znear = self.model.vis.map.znear
            zfar = self.model.vis.map.zfar
            depth_norm = np.clip((depth - znear) / (zfar - znear), 0.0, 1.0)
            return depth_norm.astype(np.float32)
        except TypeError:
            # 旧版MuJoCo不支持depth参数，回退到RGB亮度近似
            img = self._renderer.render()
            gray = 0.299*img[..., 0] + 0.587*img[..., 1] + 0.114*img[..., 2]
            return (1.0 - gray.astype(np.float32)/255.0)

    def close(self):
        if self._renderer is not None:
            try: self._renderer.close()
            except Exception: pass
            self._renderer = None
