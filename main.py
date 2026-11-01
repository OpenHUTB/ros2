#!/usr/bin/env python3
"""人形机器人端到端神经网络图像控制模型 - 整合版本

本文件整合了：
- MuJoCo 仿真核心（G1 人形机器人 + 头部摄像头 + 路点场景）
- 摄像头仿真（RGB / 深度图渲染）
- 端到端 CNN 模型（VisualEncoder + StateEncoder + FusionHead）
- 专家 PID 控制器 + 随机控制器（行为克隆数据源）
- ROS2 节点（仿真节点 sim_node、策略推理节点 policy_node、键盘遥控节点）
"""
from __future__ import annotations

import math
import os
import sys
import time
import threading

import numpy as np
import mujoco
import torch
import torch.nn as nn

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


# ============================================================================
# 常量定义
# ============================================================================

# 腿部关节名称
LEG_JOINTS = [
    "left_hip_pitch", "left_knee", "left_ankle",
    "right_hip_pitch", "right_knee", "right_ankle",
]
N_LEG_JOINTS = len(LEG_JOINTS)

# 站立默认关节角度（弧度）
NOMINAL_LEG_POSE = np.array([0.0, 0.3, -0.15, 0.0, 0.3, -0.15], dtype=np.float64)

# 关节限位
LEG_QMIN = np.array([-1.5, 0.0, -0.8, -1.5, 0.0, -0.8], dtype=np.float64)
LEG_QMAX = np.array([0.5, 2.0, 0.8, 0.5, 2.0, 0.8], dtype=np.float64)

# 路点名称
WAYPOINT_NAMES = ["wp0", "wp1", "wp2", "wp3"]

# 键盘遥控参数
MAX_LINEAR_VEL = 0.5
MAX_ANGULAR_VEL = 1.0


# ============================================================================
# MuJoCo 场景 XML（内嵌回退模型）
# ============================================================================

DEFAULT_MODEL_XML = """
<mujoco model="g1_end_to_end_scene">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <default>
    <geom condim="6" friction="0.8 0.02 0.01"/>
    <joint damping="0.5"/>
  </default>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
    <global azimuth="150" elevation="-20"/>
  </visual>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1" diffuse="1 1 1"/>
    <geom name="floor" type="plane" size="20 20 0.1" rgba="0.8 0.8 0.8 1"/>

    <!-- 路点标记（彩色球体） -->
    <body name="wp0" pos="2.0 0.0 0.05">
      <geom type="sphere" size="0.1" rgba="1 0 0 0.8" contype="0" conaffinity="0"/>
    </body>
    <body name="wp1" pos="4.0 1.0 0.05">
      <geom type="sphere" size="0.1" rgba="0 1 0 0.8" contype="0" conaffinity="0"/>
    </body>
    <body name="wp2" pos="3.0 -1.5 0.05">
      <geom type="sphere" size="0.1" rgba="0 0 1 0.8" contype="0" conaffinity="0"/>
    </body>
    <body name="wp3" pos="5.0 0.5 0.05">
      <geom type="sphere" size="0.1" rgba="1 1 0 0.8" contype="0" conaffinity="0"/>
    </body>

    <body name="pelvis" pos="0 0 0.75">
      <freejoint name="root"/>
      <geom name="pelvis_geom" type="box" size="0.1 0.08 0.05"
            rgba="0.2 0.4 0.8 1" mass="5"/>
      <camera name="head_cam" pos="0.15 0 0.35"
              xyaxes="0 -1 0 0.3 0 0.95" fovy="60"/>

      <body name="torso" pos="0 0 0.15">
        <geom type="box" size="0.08 0.06 0.12" rgba="0.3 0.5 0.9 1" mass="3"/>
      </body>

      <body name="left_thigh" pos="0.08 -0.05 -0.05">
        <joint name="left_hip_pitch" type="hinge" axis="0 1 0"
               range="-1.5 0.5"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04" mass="2"/>
        <body name="left_shin" pos="0 0 -0.3">
          <joint name="left_knee" type="hinge" axis="0 1 0"
                 range="0 2.0"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.035" mass="1.5"/>
          <body name="left_foot" pos="0 0 -0.3">
            <joint name="left_ankle" type="hinge" axis="0 1 0"
                   range="-0.8 0.8"/>
            <geom type="box" size="0.06 0.03 0.02" mass="0.5"/>
          </body>
        </body>
      </body>

      <body name="right_thigh" pos="-0.08 -0.05 -0.05">
        <joint name="right_hip_pitch" type="hinge" axis="0 1 0"
               range="-1.5 0.5"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04" mass="2"/>
        <body name="right_shin" pos="0 0 -0.3">
          <joint name="right_knee" type="hinge" axis="0 1 0"
                 range="0 2.0"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.035" mass="1.5"/>
          <body name="right_foot" pos="0 0 -0.3">
            <joint name="right_ankle" type="hinge" axis="0 1 0"
                   range="-0.8 0.8"/>
            <geom type="box" size="0.06 0.03 0.02" mass="0.5"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="left_hip_pitch_act" joint="left_hip_pitch" kp="100" kv="2"/>
    <position name="left_knee_act"      joint="left_knee"      kp="150" kv="3"/>
    <position name="left_ankle_act"     joint="left_ankle"     kp="40"  kv="1"/>
    <position name="right_hip_pitch_act" joint="right_hip_pitch" kp="100" kv="2"/>
    <position name="right_knee_act"      joint="right_knee"      kp="150" kv="3"/>
    <position name="right_ankle_act"     joint="right_ankle"     kp="40"  kv="1"/>
  </actuator>
</mujoco>
"""


# ============================================================================
# 神经网络模型定义
# ============================================================================

class VisualEncoder(nn.Module):
    """CNN 视觉编码器：84x84 RGB 图像 → 4608 维特征向量

    架构：
      Conv2d(3->32, k=5, s=2, p=2) + BN + ReLU + MaxPool(2)
      Conv2d(32->64, k=3, s=2, p=1) + BN + ReLU + MaxPool(2)
      Conv2d(64->128, k=3, s=2, p=1) + BN + ReLU + MaxPool(2)
      AdaptiveAvgPool2d((6, 6))
      输出: 128 * 6 * 6 = 4608
    """

    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            # 第 1 层: 3 -> 32
            nn.Conv2d(3, 32, kernel_size=5, stride=2, padding=2),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            # 第 2 层: 32 -> 64
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            # 第 3 层: 64 -> 128
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),
            # 自适应池化保证输出 6x6
            nn.AdaptiveAvgPool2d((6, 6)),
        )
        self.output_dim = 128 * 6 * 6  # 4608

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """x: (B, 3, 84, 84) -> (B, 4608)"""
        return self.features(x).flatten(1)


class StateEncoder(nn.Module):
    """状态编码器：6 维本体感知状态 → 32 维特征向量

    输入: (B, 6) 关节角度
    输出: (B, 32) 状态特征
    """

    def __init__(self, state_dim: int = 6, hidden_dim: int = 32):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(inplace=True),
        )
        self.output_dim = hidden_dim

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """x: (B, 6) -> (B, 32)"""
        return self.net(x)


class EndToEndCNN(nn.Module):
    """端到端 CNN 策略网络：图像 + 状态 → 关节目标

    架构：
      VisualEncoder: 图像 -> 4608 维视觉特征
      StateEncoder:  状态 -> 32 维状态特征
      FusionHead:    concat(4608, 32) -> FC(256) -> FC(128) -> FC(6)

    输入: image (B, 3, 84, 84), state (B, 6)
    输出: action (B, 6) 关节位置目标（弧度）
    """

    def __init__(
        self,
        action_dim: int = 6,
        state_dim: int = 6,
        hidden1: int = 256,
        hidden2: int = 128,
    ):
        super().__init__()
        self.visual_encoder = VisualEncoder()
        self.state_encoder = StateEncoder(state_dim=state_dim)

        fusion_in = self.visual_encoder.output_dim + self.state_encoder.output_dim

        # 融合网络
        self.fusion = nn.Sequential(
            nn.Linear(fusion_in, hidden1),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(hidden1, hidden2),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(hidden2, action_dim),
        )

        self.action_dim = action_dim

    def forward(
        self, image: torch.Tensor, state: torch.Tensor
    ) -> torch.Tensor:
        """前向传播

        Args:
            image: (B, 3, 84, 84) RGB 图像，归一化到 [0, 1]
            state: (B, 6) 本体感知状态

        Returns:
            action: (B, 6) 关节位置目标（弧度）
        """
        vis_feat = self.visual_encoder(image)
        state_feat = self.state_encoder(state)
        fused = torch.cat([vis_feat, state_feat], dim=1)
        return self.fusion(fused)

    def predict(self, image: torch.Tensor, state: torch.Tensor) -> torch.Tensor:
        """推理接口（无梯度）"""
        self.eval()
        with torch.no_grad():
            return self.forward(image, state)


def build_e2e_model() -> EndToEndCNN:
    """工厂函数：创建新的 EndToEndCNN 模型"""
    return EndToEndCNN()


def load_e2e_model(path: str) -> EndToEndCNN:
    """从文件加载训练好的 EndToEndCNN 模型"""
    model = build_e2e_model()
    if path.endswith(".pt") or path.endswith(".pth"):
        try:
            scripted = torch.jit.load(path)
            return scripted
        except Exception:
            state = torch.load(path, map_location="cpu")
            if isinstance(state, dict) and "model_state_dict" in state:
                model.load_state_dict(state["model_state_dict"])
            else:
                model.load_state_dict(state)
    return model


# ============================================================================
# 摄像头仿真
# ============================================================================

class CameraSim:
    """摄像头仿真：从 MuJoCo 渲染 RGB 和深度图

    摄像头安装在 G1 人形机器人头部，朝向前方。
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        width: int = 84,
        height: int = 84,
        camera_name: str | None = None,
        pelvis_body: str = "pelvis",
        camera_offset: np.ndarray | None = None,
    ):
        self.model = model
        self.width = width
        self.height = height
        self.renderer = mujoco.Renderer(model, height=height, width=width)

        self.pelvis_id = model.body(pelvis_body).id
        if camera_offset is None:
            camera_offset = np.array([0.15, 0.0, 0.35])
        self.camera_offset = camera_offset

        self._cam_name = camera_name
        if (
            camera_name
            and mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name) >= 0
        ):
            self._cam_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
            )
            self._use_fixed_cam = True
        else:
            self._use_fixed_cam = False
            self._cam_id = -1

    def capture(self, data: mujoco.MjData) -> np.ndarray:
        """捕获 RGB 图像，返回 uint8 数组 [H, W, 3]"""
        if self._use_fixed_cam:
            self.renderer.update_scene(data, camera=self._cam_name)
        else:
            cam = mujoco.MjvCamera()
            pelvis_pos = data.xpos[self.pelvis_id]
            offset_world = (
                data.xmat[self.pelvis_id].reshape(3, 3) @ self.camera_offset
            )
            cam.lookat[:] = pelvis_pos + offset_world + np.array([1.0, 0.0, 0.0])
            cam.distance = 0.01
            cam.azimuth = 0.0
            cam.elevation = 0.0
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.renderer.update_scene(data, camera=cam)

        frame = self.renderer.render()
        return frame.copy()

    def capture_depth(self, data: mujoco.MjData) -> np.ndarray:
        """捕获深度图，返回 float32 数组 [H, W]"""
        self.renderer.enable_depth_rendering()
        if self._use_fixed_cam:
            self.renderer.update_scene(data, camera=self._cam_name)
        else:
            cam = mujoco.MjvCamera()
            pelvis_pos = data.xpos[self.pelvis_id]
            offset_world = (
                data.xmat[self.pelvis_id].reshape(3, 3) @ self.camera_offset
            )
            cam.lookat[:] = pelvis_pos + offset_world + np.array([1.0, 0.0, 0.0])
            cam.distance = 0.01
            cam.azimuth = 0.0
            cam.elevation = 0.0
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.renderer.update_scene(data, camera=cam)
        depth = self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        return depth


# ============================================================================
# MuJoCo 仿真核心
# ============================================================================

def _resolve_model_path(model_path: str) -> str:
    """多级模型路径解析"""
    if model_path and os.path.isfile(model_path):
        return model_path
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.realpath(os.path.join(here, "models", "scene_e2e.xml"))
    if os.path.isfile(candidate):
        return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("humanoid_end_to_end")
        candidate = os.path.join(share, "models", "scene_e2e.xml")
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    return model_path


class G1EndToEndSim:
    """MuJoCo 仿真核心：G1 人形机器人 + 摄像头，用于端到端神经网络控制

    纯 Python + MuJoCo，无 ROS 依赖。
    """

    SIM_DT = 0.004
    CTRL_DT = 0.02
    N_SUBSTEPS = int(round(CTRL_DT / SIM_DT))

    def __init__(self, model_path: str = ""):
        resolved = _resolve_model_path(model_path)
        if os.path.isfile(resolved):
            with open(resolved, encoding="utf-8") as f:
                xml = f.read()
            self.model = mujoco.MjModel.from_xml_string(xml)
        else:
            self.model = mujoco.MjModel.from_xml_string(DEFAULT_MODEL_XML)

        self.model.opt.timestep = self.SIM_DT
        self.data = mujoco.MjData(self.model)

        # 摄像头
        self.camera = CameraSim(self.model, width=84, height=84)

        # 刚体 / 关节 ID
        self._pelvis_id = self.model.body("pelvis").id
        self._joint_names = list(LEG_JOINTS)
        self._joint_ids = []
        for name in self._joint_names:
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if jid >= 0:
                self._joint_ids.append(jid)
        self._qpos_adr = np.array(
            [self.model.jnt_qposadr[j] for j in self._joint_ids]
        )
        self._dof_adr = np.array(
            [self.model.jnt_dofadr[j] for j in self._joint_ids]
        )

        # 路点 ID
        self._wp_ids = []
        for wp_name in WAYPOINT_NAMES:
            bid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, wp_name
            )
            if bid >= 0:
                self._wp_ids.append(bid)

        # 关节名称列表（用于 ROS 发布）
        self.joint_names = list(self._joint_names)

        self.reset()

    # ----- 重置 / 步进 -----

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        """重置仿真到站立姿态 (x, y, yaw)"""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = x
        self.data.qpos[1] = y
        self.data.qpos[2] = 0.75
        qw = np.cos(yaw / 2)
        qz = np.sin(yaw / 2)
        self.data.qpos[3:7] = [qw, 0.0, 0.0, qz]
        if len(self._qpos_adr) == len(NOMINAL_LEG_POSE):
            self.data.qpos[self._qpos_adr] = NOMINAL_LEG_POSE
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def step(self, joint_targets: np.ndarray | None = None):
        """以 CTRL_DT 为步长推进物理仿真"""
        if joint_targets is not None:
            n = min(len(joint_targets), len(self.data.ctrl))
            self.data.ctrl[:n] = joint_targets[:n]
        for _ in range(self.N_SUBSTEPS):
            mujoco.mj_step(self.model, self.data)

    # ----- 观测接口 -----

    def capture_image(self) -> np.ndarray:
        """从头部摄像头捕获 84x84 RGB 图像"""
        return self.camera.capture(self.data)

    def capture_depth(self) -> np.ndarray:
        """从头部摄像头捕获 84x84 深度图"""
        return self.camera.capture_depth(self.data)

    def get_position(self) -> np.ndarray:
        """返回骨盆在世界坐标系中的位置 [x, y, z]"""
        return self.data.qpos[:3].copy()

    def get_orientation(self) -> np.ndarray:
        """返回骨盆四元数 [w, x, y, z]"""
        return self.data.qpos[3:7].copy()

    def get_yaw(self) -> float:
        """从四元数提取偏航角"""
        w, x, y, z = self.data.qpos[3:7]
        return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    def get_pelvis_height(self) -> float:
        """返回骨盆离地高度"""
        return float(self.data.xpos[self._pelvis_id, 2])

    def has_fallen(self) -> bool:
        """检测机器人是否跌倒（高度 < 0.4m）"""
        return self.get_pelvis_height() < 0.4

    def joint_pos(self) -> np.ndarray:
        """返回当前腿部关节位置（6 维）"""
        return self.data.qpos[self._qpos_adr].copy()

    def joint_vel(self) -> np.ndarray:
        """返回当前腿部关节速度（6 维）"""
        return self.data.qvel[self._dof_adr].copy()

    def get_observation(self) -> np.ndarray:
        """返回 6 维本体感知状态"""
        return self.joint_pos()

    def get_full_state(self) -> np.ndarray:
        """返回 6 维状态向量（用于 E2E 模型状态编码器）"""
        qpos = self.joint_pos()
        return qpos[:6].copy()

    # ----- 路点接口 -----

    def get_waypoint_positions(self) -> np.ndarray:
        """返回 Nx3 路点位置数组"""
        if not self._wp_ids:
            return np.zeros((0, 3))
        return self.data.xpos[self._wp_ids].copy()

    def get_nearest_waypoint(self) -> tuple[int, np.ndarray, float]:
        """返回 (索引, 位置, 距离) 最近路点"""
        wps = self.get_waypoint_positions()
        if len(wps) == 0:
            return -1, np.zeros(3), float("inf")
        pos = self.get_position()
        dists = np.linalg.norm(wps[:, :2] - pos[:2], axis=1)
        idx = int(np.argmin(dists))
        return idx, wps[idx], float(dists[idx])

    def get_waypoint_direction(self) -> np.ndarray:
        """返回从机器人到最近路点的单位方向向量（2D）"""
        _, wp_pos, dist = self.get_nearest_waypoint()
        if dist < 1e-6:
            return np.array([1.0, 0.0])
        pos = self.get_position()
        dx = wp_pos[0] - pos[0]
        dy = wp_pos[1] - pos[1]
        norm = np.sqrt(dx * dx + dy * dy)
        return np.array([dx / norm, dy / norm])

    # ----- 渲染 -----

    def render(self, width: int = 640, height: int = 480) -> np.ndarray:
        """渲染完整场景视图为 RGB 数组"""
        renderer = mujoco.Renderer(self.model, height=height, width=width)
        cam = mujoco.MjvCamera()
        pos = self.data.qpos[:3]
        cam.lookat[:] = [pos[0], pos[1], 0.5]
        cam.distance = 3.0
        cam.azimuth = 135.0
        cam.elevation = -15.0
        renderer.update_scene(self.data, camera=cam)
        frame = renderer.render().copy()
        renderer.close()
        return frame


# ============================================================================
# 专家控制器
# ============================================================================

class ExpertController:
    """基于规则的专家控制器（PID + 深度避障）

    根据以下信息计算 6 维关节位置目标：
      - 前向速度指令 -> 髋俯仰角
      - 障碍物距离（深度相机） -> 转向 / 步态调节
      - 平衡 -> 膝 / 踝关节高度误差补偿

    该控制器用于生成行为克隆的专家演示数据。
    """

    def __init__(
        self,
        target_vx: float = 0.2,
        kp_height: float = 2.0,
        kd_height: float = 0.5,
        target_height: float = 0.75,
        max_hip_pitch: float = 0.4,
        max_knee: float = 0.6,
        gait_freq: float = 1.5,
    ):
        self.target_vx = target_vx
        self.kp_height = kp_height
        self.kd_height = kd_height
        self.target_height = target_height
        self.max_hip_pitch = max_hip_pitch
        self.max_knee = max_knee
        self.gait_freq = gait_freq

        self._phase = 0.0
        self._prev_height_error = 0.0
        self._prev_action = NOMINAL_LEG_POSE.copy()

    def compute_action(
        self,
        sim: G1EndToEndSim,
        cmd_vx: float | None = None,
        cmd_wz: float = 0.0,
        dt: float = 0.02,
    ) -> np.ndarray:
        """计算 6 维关节位置目标

        Args:
            sim: 仿真实例
            cmd_vx: 前向速度指令（None 使用默认值）
            cmd_wz: 转向角速度指令（rad/s）
            dt: 控制步长

        Returns:
            action: (6,) 关节位置目标（弧度）
        """
        if cmd_vx is None:
            cmd_vx = self.target_vx

        # 推进步态相位
        self._phase = (self._phase + self.gait_freq * dt) % 1.0
        phase_sin = np.sin(2.0 * np.pi * self._phase)
        phase_cos = np.cos(2.0 * np.pi * self._phase)

        # --- 高度平衡（PID 控制骨盆高度误差） ---
        current_height = sim.get_pelvis_height()
        height_error = self.target_height - current_height
        height_deriv = (height_error - self._prev_height_error) / max(dt, 1e-6)
        self._prev_height_error = height_error

        balance_correction = (
            self.kp_height * height_error + self.kd_height * height_deriv
        )

        # --- 前向速度 -> 髋俯仰角振荡 ---
        speed_ratio = np.clip(cmd_vx / max(self.target_vx, 0.01), 0.0, 2.0)
        hip_amplitude = self.max_hip_pitch * speed_ratio

        # 左右髋 180 度相位差
        left_hip_pitch = NOMINAL_LEG_POSE[0] + hip_amplitude * phase_sin
        right_hip_pitch = NOMINAL_LEG_POSE[3] - hip_amplitude * phase_sin

        # --- 转向 -> 差分髋部调节 ---
        turn_modulation = np.clip(cmd_wz * 0.3, -0.2, 0.2)
        left_hip_pitch += turn_modulation * phase_cos
        right_hip_pitch -= turn_modulation * phase_cos

        # --- 膝关节弯曲（支撑体重 + 平衡） ---
        knee_base = NOMINAL_LEG_POSE[1]
        knee_balance = np.clip(balance_correction * 0.3, -0.2, 0.3)
        knee_oscillation = 0.05 * abs(phase_sin) * speed_ratio

        left_knee = knee_base + knee_balance + knee_oscillation
        right_knee = knee_base + knee_balance + knee_oscillation

        # --- 踝关节补偿 ---
        ankle_base = NOMINAL_LEG_POSE[2]
        ankle_compensation = -0.3 * knee_balance
        ankle_oscillation = -0.02 * phase_sin * speed_ratio

        left_ankle = ankle_base + ankle_compensation + ankle_oscillation
        right_ankle = ankle_base + ankle_compensation - ankle_oscillation

        # --- 深度相机避障 ---
        try:
            depth = sim.capture_depth()
            h, w = depth.shape
            front_region = depth[h // 2:, w // 4:3 * w // 4]
            min_front_depth = float(np.min(front_region))

            if min_front_depth < 1.0:
                avoidance_factor = np.clip(min_front_depth / 1.0, 0.0, 1.0)
                left_hip_pitch = NOMINAL_LEG_POSE[0] + (
                    left_hip_pitch - NOMINAL_LEG_POSE[0]
                ) * avoidance_factor
                right_hip_pitch = NOMINAL_LEG_POSE[3] + (
                    right_hip_pitch - NOMINAL_LEG_POSE[3]
                ) * avoidance_factor
        except Exception:
            pass

        # 组装动作
        action = np.array([
            left_hip_pitch, left_knee, left_ankle,
            right_hip_pitch, right_knee, right_ankle,
        ], dtype=np.float64)

        # 裁剪到关节限位
        action = np.clip(action, LEG_QMIN, LEG_QMAX)

        # 低通滤波平滑
        alpha = 0.7
        action = alpha * action + (1.0 - alpha) * self._prev_action
        self._prev_action = action.copy()

        return action

    def reset(self):
        """重置内部状态"""
        self._phase = 0.0
        self._prev_height_error = 0.0
        self._prev_action = NOMINAL_LEG_POSE.copy()


class RandomController:
    """随机动作控制器（基线对比用）"""

    def __init__(self, seed: int = 42):
        self._rng = np.random.default_rng(seed)

    def compute_action(
        self,
        sim: G1EndToEndSim,
        cmd_vx: float = 0.0,
        cmd_wz: float = 0.0,
        dt: float = 0.02,
    ) -> np.ndarray:
        """返回关节限位内的随机目标"""
        return self._rng.uniform(LEG_QMIN, LEG_QMAX)

    def reset(self):
        pass


# ============================================================================
# 辅助函数
# ============================================================================

def _yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    """从四元数提取偏航角"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ============================================================================
# ROS2 节点
# ============================================================================

class SimNode(Node):
    """仿真节点：发布 /camera/image_raw, /odom, /joint_states, /humanoid/obs
    订阅 /humanoid/joint_targets, /cmd_vel
    """

    def __init__(self):
        super().__init__("humanoid_e2e_sim_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("sim_rate", 250.0)
        self.declare_parameter("image_rate", 10.0)
        self.declare_parameter("publish_tf", True)

        model_path = self.get_parameter("model_path").value
        sim_rate = float(self.get_parameter("sim_rate").value)
        image_rate = float(self.get_parameter("image_rate").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.sim = G1EndToEndSim(model_path=model_path)

        # CV Bridge
        if HAS_CV_BRIDGE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge 不可用；图像发布使用原始 numpy")

        # 发布者
        self.pub_image = self.create_publisher(Image, "/camera/image_raw", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_obs = self.create_publisher(Float64MultiArray, "/humanoid/obs", 10)

        # 订阅者
        self.create_subscription(
            Float64MultiArray, "/humanoid/joint_targets", self._on_targets, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # TF 广播器
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        # 状态
        self.joint_targets = NOMINAL_LEG_POSE.copy()
        self.cmd_vel = np.zeros(2)  # [vx, wz]
        self._sim_dt = 1.0 / sim_rate
        self._image_dt = 1.0 / image_rate
        self._step_count = 0
        self._image_count = 0

        # 定时器
        self.timer_sim = self.create_timer(self._sim_dt, self._sim_step)
        self.timer_image = self.create_timer(self._image_dt, self._publish_image)

        self.get_logger().info(
            f"E2E SimNode 启动: model={model_path}, "
            f"sim_rate={sim_rate:.0f}Hz, image_rate={image_rate:.0f}Hz"
        )

    # ----- 回调 -----

    def _on_targets(self, msg: Float64MultiArray) -> None:
        """接收 6 维关节位置目标"""
        data = np.asarray(msg.data, dtype=np.float64)
        if data.size >= len(LEG_JOINTS):
            self.joint_targets = data[: len(LEG_JOINTS)]

    def _on_cmd_vel(self, msg: Twist) -> None:
        """接收速度指令"""
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.angular.z

    # ----- 主循环 -----

    def _sim_step(self) -> None:
        """推进物理仿真并发布状态"""
        self.sim.step(self.joint_targets)
        self._step_count += 1

        # 以 20 Hz 发布里程计和关节状态
        publish_rate = 20.0
        if self._step_count % max(1, int(1.0 / (publish_rate * self._sim_dt))) == 0:
            self._publish_odom()
            self._publish_joints()
            self._publish_obs()

    def _publish_image(self) -> None:
        """捕获并发布摄像头图像"""
        frame = self.sim.capture_image()
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "head_cam"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.pub_image.publish(msg)
        self._image_count += 1

    def _publish_odom(self) -> None:
        """发布里程计消息"""
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        qpos = self.sim.data.qpos
        odom.pose.pose.position.x = float(qpos[0])
        odom.pose.pose.position.y = float(qpos[1])
        odom.pose.pose.position.z = float(qpos[2])
        odom.pose.pose.orientation.w = float(qpos[3])
        odom.pose.pose.orientation.x = float(qpos[4])
        odom.pose.pose.orientation.y = float(qpos[5])
        odom.pose.pose.orientation.z = float(qpos[6])

        # 体坐标系速度
        yaw = _yaw_from_quat(
            float(qpos[3]), float(qpos[4]), float(qpos[5]), float(qpos[6])
        )
        vx_world = float(self.sim.data.qvel[0])
        vy_world = float(self.sim.data.qvel[1])
        vx_body = vx_world * math.cos(yaw) + vy_world * math.sin(yaw)
        vy_body = -vx_world * math.sin(yaw) + vy_world * math.cos(yaw)
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.angular.z = float(self.sim.data.qvel[5])

        self.pub_odom.publish(odom)

        # TF
        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = float(qpos[0])
            tf.transform.translation.y = float(qpos[1])
            tf.transform.translation.z = float(qpos[2])
            tf.transform.rotation.w = float(qpos[3])
            tf.transform.rotation.x = float(qpos[4])
            tf.transform.rotation.y = float(qpos[5])
            tf.transform.rotation.z = float(qpos[6])
            self.tf_broadcaster.sendTransform(tf)

    def _publish_joints(self) -> None:
        """发布关节状态"""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.sim.joint_names)
        js.position = self.sim.data.qpos[self.sim._qpos_adr].tolist()
        js.velocity = self.sim.data.qvel[self.sim._dof_adr].tolist()
        js.effort = []
        self.pub_joint.publish(js)

    def _publish_obs(self) -> None:
        """发布 6 维观测（用于 E2E 模型）"""
        obs = self.sim.get_full_state()
        msg = Float64MultiArray()
        msg.data = obs.astype(float).tolist()
        self.pub_obs.publish(msg)


class PolicyNode(Node):
    """策略推理节点：订阅 /camera/image_raw 和 /humanoid/obs，
    运行端到端 CNN 模型，发布 /humanoid/joint_targets

    推理流程：image (84x84x3) + state (6,) -> CNN -> joint targets (6,)
    """

    def __init__(self):
        super().__init__("humanoid_e2e_policy_node")

        self.declare_parameter("weights", "")
        self.declare_parameter("use_expert_fallback", False)
        self.declare_parameter("inference_rate", 10.0)
        self.declare_parameter("device", "cpu")

        weights_path = self.get_parameter("weights").value
        self.use_expert_fallback = bool(
            self.get_parameter("use_expert_fallback").value
        )
        inference_rate = float(self.get_parameter("inference_rate").value)
        device_str = self.get_parameter("device").value
        self.device = torch.device(
            device_str if torch.cuda.is_available() or device_str == "cpu"
            else "cpu"
        )

        # 加载模型
        self.model = build_e2e_model()
        if weights_path and os.path.isfile(weights_path):
            self.model = load_e2e_model(weights_path)
            self.get_logger().info(f"已加载权重: {weights_path}")
        else:
            self.get_logger().warn(
                "未找到权重文件；使用未训练模型。请先运行训练脚本。"
            )
        self.model.to(self.device)
        self.model.eval()

        # 最新观测状态
        self._latest_image = None
        self._latest_state = np.zeros(6, dtype=np.float32)
        self._latency_ms = 0.0

        # 订阅者
        self.sub_image = self.create_subscription(
            Image, "/camera/image_raw", self._on_image, 10
        )
        self.sub_obs = self.create_subscription(
            Float64MultiArray, "/humanoid/obs", self._on_obs, 10
        )

        # 发布者
        self.pub_action = self.create_publisher(
            Float64MultiArray, "/humanoid/joint_targets", 10
        )

        # 推理定时器
        self._inference_dt = 1.0 / inference_rate
        self.timer = self.create_timer(self._inference_dt, self._inference_step)

        self.get_logger().info(
            f"E2E PolicyNode 启动: device={self.device}, "
            f"rate={inference_rate:.0f}Hz, "
            f"expert_fallback={self.use_expert_fallback}"
        )

    def _on_image(self, msg: Image) -> None:
        """存储最新摄像头图像"""
        try:
            h, w = msg.height, msg.width
            if msg.encoding == "rgb8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            elif msg.encoding == "bgr8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                img = img[:, :, ::-1].copy()
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            self._latest_image = img
        except Exception as e:
            self.get_logger().debug(f"图像解码错误: {e}")

    def _on_obs(self, msg: Float64MultiArray) -> None:
        """存储最新观测状态"""
        data = np.asarray(msg.data, dtype=np.float32)
        if data.size >= 6:
            self._latest_state = data[:6]

    def _inference_step(self) -> None:
        """执行一次推理并发布关节目标"""
        if self._latest_image is None:
            return

        t0 = time.perf_counter()

        # 预处理图像: (84, 84, 3) -> (1, 3, 84, 84) float [0, 1]
        img = self._latest_image
        if img.shape[0] != 84 or img.shape[1] != 84:
            img = img[:84, :84, :] if img.shape[0] >= 84 and img.shape[1] >= 84 else img

        img_tensor = (
            torch.from_numpy(img.astype(np.float32) / 255.0)
            .permute(2, 0, 1)
            .unsqueeze(0)
            .to(self.device)
        )
        state_tensor = (
            torch.from_numpy(self._latest_state)
            .unsqueeze(0)
            .to(self.device)
        )

        # 前向推理
        with torch.no_grad():
            action = self.model(img_tensor, state_tensor)

        action_np = action[0].cpu().numpy()

        t1 = time.perf_counter()
        self._latency_ms = (t1 - t0) * 1000.0

        # 发布
        msg = Float64MultiArray()
        msg.data = action_np.astype(float).tolist()
        self.pub_action.publish(msg)

    def get_latency_ms(self) -> float:
        """返回最近一次推理延迟（毫秒）"""
        return self._latency_ms


class KeyboardTeleopNode(Node):
    """键盘遥控节点：WASD 键 -> /cmd_vel (Twist)

    控制：
      W / S : 前进 / 后退 (linear.x)
      A / D : 左转 / 右转 (angular.z)
      X     : 停止（零速度）
      Q     : 退出
    """

    def __init__(self):
        super().__init__("e2e_keyboard_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._running = True
        self._thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            "键盘遥控启动。W/S=前进/后退, A/D=左转/右转, X=停止, Q=退出"
        )

    def _keyboard_loop(self):
        try:
            if sys.platform == "win32":
                self._win_loop()
            else:
                self._linux_loop()
        except Exception as e:
            self.get_logger().error(f"键盘循环错误: {e}")

    def _linux_loop(self):
        import tty
        import termios

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running:
                ch = sys.stdin.read(1)
                if ch == "q" or ch == "Q":
                    self._running = False
                    rclpy.shutdown()
                    break
                self._process_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _win_loop(self):
        import msvcrt

        while self._running:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                if ch == "q" or ch == "Q":
                    self._running = False
                    rclpy.shutdown()
                    break
                self._process_key(ch)

    def _process_key(self, key: str):
        msg = Twist()
        if key == "w":
            msg.linear.x = MAX_LINEAR_VEL
        elif key == "s":
            msg.linear.x = -MAX_LINEAR_VEL
        elif key == "a":
            msg.angular.z = MAX_ANGULAR_VEL
        elif key == "d":
            msg.angular.z = -MAX_ANGULAR_VEL
        elif key == "x":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            return
        self.pub.publish(msg)


# ============================================================================
# 主函数
# ============================================================================

def main(args=None):
    """启动所有 ROS2 节点：仿真、策略推理、键盘遥控"""
    rclpy.init(args=args)

    sim_node = SimNode()
    policy_node = PolicyNode()
    teleop_node = KeyboardTeleopNode()

    from concurrent.futures import ThreadPoolExecutor
    executor = ThreadPoolExecutor(max_workers=3)

    try:
        executor.submit(rclpy.spin, sim_node)
        executor.submit(rclpy.spin, policy_node)
        executor.submit(rclpy.spin, teleop_node)

        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    finally:
        sim_node.destroy_node()
        policy_node.destroy_node()
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
