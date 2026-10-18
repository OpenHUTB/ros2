"""
sim.py —— MuJoCo 人形机器人物理仿真封装
=========================================

职责：
  * 加载 humanoid_camera.xml 模型；
  * 提供 reset/step/apply_push/render_front/fallen 等基础接口；
  * 站姿 PD 控制（让机器人在无扰动下稳定站立）；
  * 前视相机渲染为灰度图（用于神经网络输入）。

本模块为 standalone 实现，不依赖 humanoid_push_recovery（该模块尚未合并到
upstream/master，本 PR 不引用未合并代码）。
"""

from __future__ import annotations

import os
import numpy as np

try:
    import mujoco
except (ImportError, OSError, RuntimeError):  # pragma: no cover
    # mujoco 可能因 OpenGL / 驱动问题抛 OSError/RuntimeError；
    # 此处置 None，让只依赖常量表的模块（controller / dataset 标签）仍可 import。
    mujoco = None


# 关节名顺序（与 XML <actuator> 一致）
JOINT_NAMES = [
    "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_pitch_l", "ankle_roll_l",
    "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_pitch_r", "ankle_roll_r",
    "shoulder_pitch_l", "elbow_l",
    "shoulder_pitch_r", "elbow_r",
    "neck",
]

# 腿部关节索引（需要 PD 站姿控制）
LEG_JOINT_IDX = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

# 默认站姿目标（弧度）：近似直立，手臂自然下垂
DEFAULT_STAND_TARGET = np.array([
    0.0,   # hip_roll_l
    0.08,  # hip_pitch_l
    -0.16, # knee_l
    -0.08, # ankle_pitch_l
    0.0,   # ankle_roll_l
    0.0,   # hip_roll_r
    0.08,  # hip_pitch_r
    -0.16, # knee_r
    -0.08, # ankle_pitch_r
    0.0,   # ankle_roll_r
    0.0,   # shoulder_pitch_l
    -1.4,  # elbow_l
    0.0,   # shoulder_pitch_r
    -1.4,  # elbow_r
    0.0,   # neck
], dtype=np.float64)


class VisionFallSim:
    """人形机器人物理仿真（视觉跌倒预测用）。"""

    def __init__(self, model_path: str, kp: float = 60.0, kd: float = 6.0,
                 image_h: int = 64, image_w: int = 64):
        if mujoco is None:
            raise RuntimeError("mujoco 未安装，请先 pip install mujoco")
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"模型文件不存在: {model_path}")

        # 用 from_xml_string 避免 Windows 中文路径下 from_xml_path 打不开文件的问题
        with open(model_path, "r", encoding="utf-8") as f:
            xml_str = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml_str)
        self.data = mujoco.MjData(self.model)
        self.kp = float(kp)
        self.kd = float(kd)
        self.image_h = image_h
        self.image_w = image_w

        # 关节 qpos/qvel 索引（freejoint 占 7 个 qpos、6 个 qvel）
        self.qpos_joint_start = 7
        self.qvel_joint_start = 6

        # 前视相机渲染器（延迟初始化）
        self._renderer = None
        self._camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA,
                                            "front_camera")

        self.reset()

    # ------------------------------------------------------------------
    # 基础接口
    # ------------------------------------------------------------------
    def reset(self):
        """重置到初始站姿。"""
        mujoco.mj_resetData(self.model, self.data)
        # 设站姿目标（手臂自然下垂、腿微屈）
        self.data.qpos[self.qpos_joint_start:self.qpos_joint_start + 15] = \
            DEFAULT_STAND_TARGET
        mujoco.mj_forward(self.model, self.data)

    def step(self, target: np.ndarray | None = None):
        """推进一步；target 为 15 维关节目标角（None 则维持站姿）。"""
        if target is None:
            target = DEFAULT_STAND_TARGET
        qpos_j = self.data.qpos[self.qpos_joint_start:
                                self.qpos_joint_start + 15]
        qvel_j = self.data.qvel[self.qvel_joint_start:
                                self.qvel_joint_start + 15]
        # PD 力矩：tau = kp*(target - q) - kd*qvel
        tau = self.kp * (target - qpos_j) - self.kd * qvel_j
        # 限幅
        tau = np.clip(tau, -280.0, 280.0)
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)

    def apply_push(self, force_xyz: tuple[float, float, float],
                   duration_steps: int = 40):
        """对 torso 施加外力（N），持续 duration_steps 步。"""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        fx, fy, fz = force_xyz
        for _ in range(max(1, int(duration_steps))):
            self.data.xfrc_applied[body_id, :3] = [fx, fy, fz]
            self.step()
        # 清除外力
        self.data.xfrc_applied[body_id, :3] = [0.0, 0.0, 0.0]

    # ------------------------------------------------------------------
    # 状态读取
    # ------------------------------------------------------------------
    @property
    def root_z(self) -> float:
        """根节点高度（m）。"""
        return float(self.data.qpos[2])

    def fallen(self, z_threshold: float = 0.6) -> bool:
        """判断是否跌倒：根高低于阈值。"""
        return self.root_z < z_threshold

    def com_position(self) -> np.ndarray:
        """质心位置（世界坐标）。"""
        return self.data.subtree_com[0].copy()

    def com_velocity(self) -> np.ndarray:
        """质心速度（世界坐标）。"""
        # 用根速度近似质心速度（质心主要在躯干）
        return self.data.qvel[:3].copy()

    def torso_tilt(self) -> float:
        """躯干前倾/后倾角（弧度，正为前倾）。"""
        # 用根四元数估算：z 轴在 x-z 平面的投影角
        qw, qx, qy, qz = self.data.qpos[3:7]
        # 躯干 forward 方向 (x) 在世界系中的俯仰角
        # x 轴世界坐标 = (1-2(yy+zz), 2(xy-wz), 2(xz+wy))
        forward_z = 2.0 * (qx * qz + qw * qy)
        forward_x = 1.0 - 2.0 * (qy * qy + qz * qz)
        return float(np.arctan2(forward_z, forward_x))

    # ------------------------------------------------------------------
    # 视觉
    # ------------------------------------------------------------------
    def render_front(self, gray: bool = True) -> np.ndarray:
        """渲染前视相机图像，返回 (H, W) 灰度图或 (H, W, 3) 彩色图。"""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.image_h, self.image_w)
        self._renderer.update_scene(self.data, camera=self._camera_id)
        img = self._renderer.render()
        if gray:
            # 加权转灰度并归一化到 [0,1]
            gray_img = 0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]
            return gray_img.astype(np.float32) / 255.0
        return img

    def close(self):
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None
