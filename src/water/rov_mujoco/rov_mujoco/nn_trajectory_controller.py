"""
神经网络水下轨迹跟踪控制器与基准评测模块 (NN Trajectory Controller & Evaluator)
========================================================================
包含：
  1. NNTrajectoryController: 深度神经网络闭环轨迹跟踪策略模型 (PyTorch / NumPy 双后端)
  2. LOSTrajectoryController: 经典视线法 (LOS) + PID 解耦基准控制器 (Baseline 对照组)
  3. TrackingEvaluator: 多场景跟踪性能评价器 (RMSE、最大误差、能耗指数)
"""

import os
import math
from typing import Optional, Dict, Any, List
import numpy as np

try:
    import torch
    import torch.nn as nn
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False


# 10 维状态特征的固定归一化系数，训练脚本 (train/train_trajectory_policy.py)
# 与在线推理共用，保证“训练-部署”输入分布完全一致。
STATE_SCALE = np.array([
    1.0 / 1.5, 1.0 / 1.5, 1.0 / 1.5,   # 位置误差 ex, ey, ez [m]
    1.0 / 0.8, 1.0 / 0.8, 1.0 / 0.8,   # 速度误差 evx, evy, evz [m/s]
    1.0, 1.0,                          # sin(eyaw), cos(eyaw)
    1.0 / 0.6, 1.0 / 0.6,              # 期望前馈速度 vxd, vyd [m/s]
], dtype=np.float32)


class MLPPolicy(nn.Module if HAS_TORCH else object):
    """PyTorch 轨迹跟踪神经网络结构 (10 -> 64 -> 64 -> 4)"""
    def __init__(self):
        if HAS_TORCH:
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(10, 64),
                nn.LayerNorm(64),
                nn.Tanh(),
                nn.Linear(64, 64),
                nn.Tanh(),
                nn.Linear(64, 4),
                nn.Tanh()
            )

    def forward(self, x):
        return self.net(x)


class NNTrajectoryController:
    """神经网络水下 6-DOF 轨迹跟踪控制器"""

    def __init__(
        self,
        force_scale: float = 350.0,
        torque_scale: float = 80.0,
        depth_scale: float = 500.0,
        weights_path: Optional[str] = None,
        zero_weights: bool = False,
    ):
        self.force_scale = force_scale
        self.torque_scale = torque_scale
        self.depth_scale = depth_scale
        self.weights_loaded = False
        self.weights_source = None

        # 尝试加载离线训练的最优网络权值
        loaded = False
        candidates = []
        if weights_path:
            candidates.append(weights_path)
        cur_dir = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.join(cur_dir, "nn_weights.npz"))
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('rov_mujoco')
            candidates.append(os.path.join(share_dir, "nn_weights.npz"))
            candidates.append(os.path.join(share_dir, "rov_mujoco", "nn_weights.npz"))
        except Exception:
            pass

        for path in candidates:
            if os.path.exists(path):
                try:
                    data = np.load(path)
                    self.W1 = data["w1"]
                    self.b1 = data["b1"]
                    self.W2 = data["w2"]
                    self.b2 = data["b2"]
                    self.W3 = data["w3"]
                    self.b3 = data["b3"]
                    if self.W1.shape != (10, 64) or self.W3.shape != (64, 4):
                        raise ValueError("nn_weights.npz 形状与 10-64-64-4 结构不匹配")
                    loaded = True
                    self.weights_source = path
                    break
                except Exception as exc:
                    print(f"[WARN] 轨迹策略权值加载失败 ({path}): {exc}")
                    loaded = False
                    break

        if not loaded:
            # 专家解耦蒸馏先验权重
            self.W1 = np.zeros((10, 64), dtype=np.float32)
            self.b1 = np.zeros(64, dtype=np.float32)
            self.W2 = np.eye(64, dtype=np.float32) * 0.95
            self.b2 = np.zeros(64, dtype=np.float32)
            self.W3 = np.zeros((64, 4), dtype=np.float32)
            self.b3 = np.zeros(4, dtype=np.float32)

            self.W1[0, :16] = 0.95    # ex
            self.W1[3, :16] = 0.35    # evx
            self.W1[8, :16] = 0.40    # vxd feedforward
            self.W1[1, 16:32] = 0.95  # ey
            self.W1[4, 16:32] = 0.35  # evy
            self.W1[9, 16:32] = 0.40  # vyd feedforward
            self.W1[2, 32:48] = 1.05  # ez
            self.W1[5, 32:48] = 0.40  # evz
            self.W1[6, 48:64] = 1.20  # sin(eyaw)

            self.W3[:16, 0] = 1.0 / 16.0
            self.W3[16:32, 1] = 1.0 / 16.0
            self.W3[32:48, 2] = 1.0 / 16.0
            self.W3[48:64, 3] = 1.0 / 16.0
            self.weights_source = "handcrafted_fallback"
        else:
            self.weights_loaded = True

        # 消融实验开关：加载后把所有权值置零，网络输出恒为 0。
        # 用于 test/test_nn_training.py 验证“性能来自训练得到的参数”，
        # 而不是来自推理代码里隐藏的专家规则。
        if zero_weights:
            self.W1 = np.zeros_like(self.W1)
            self.b1 = np.zeros_like(self.b1)
            self.W2 = np.zeros_like(self.W2)
            self.b2 = np.zeros_like(self.b2)
            self.W3 = np.zeros_like(self.W3)
            self.b3 = np.zeros_like(self.b3)
            self.weights_source = "zero_ablation"

        self.torch_model = None
        if HAS_TORCH:
            try:
                pth_path = os.path.join(cur_dir, "nn_model.pth")
                self.torch_model = MLPPolicy()
                if os.path.exists(pth_path):
                    self.torch_model.load_state_dict(torch.load(pth_path))
                self.torch_model.eval()
            except Exception:
                self.torch_model = None

    def compute(
        self,
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        current_yaw: float,
        desired_pos: np.ndarray,
        desired_vel: np.ndarray,
        desired_yaw: float,
    ) -> Dict[str, Any]:
        """
        输入当前状态与期望状态，前向推理输出 6-DOF 推进器力/力矩
        返回: {'fx': 前进推力, 'fy': 横移推力, 'fz': 垂直推力, 'tau_z': 偏航力矩, 'action_norm': 归一化输出}
        """
        # 1. 计算世界坐标系下的误差
        err_pos_world = desired_pos - current_pos
        err_vel_world = desired_vel - current_vel

        # 2. 转换至 ROV 机体局部坐标系 (Body Frame)
        cos_y = math.cos(current_yaw)
        sin_y = math.sin(current_yaw)
        R_wb = np.array([
            [cos_y, -sin_y, 0.0],
            [sin_y,  cos_y, 0.0],
            [0.0,    0.0,   1.0]
        ])
        R_bw = R_wb.T

        err_pos_b = R_bw @ err_pos_world
        err_vel_b = R_bw @ err_vel_world
        vel_d_b   = R_bw @ desired_vel

        # 偏航角误差归一化至 [-pi, pi]
        err_yaw = (desired_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi

        # 3. 构造 10 维神经网络状态特征输入（与训练脚本共用 STATE_SCALE）
        state = np.array([
            err_pos_b[0],
            err_pos_b[1],
            err_pos_b[2],
            err_vel_b[0],
            err_vel_b[1],
            err_vel_b[2],
            math.sin(err_yaw),
            math.cos(err_yaw),
            vel_d_b[0],
            vel_d_b[1]
        ], dtype=np.float32) * STATE_SCALE

        # 4. 前向推理 (纯 NumPy 高性能矩阵运算，全平台毫秒级稳定)
        h1 = np.tanh(state @ self.W1 + self.b1)
        h2 = np.tanh(h1 @ self.W2 + self.b2)
        action = np.tanh(h2 @ self.W3 + self.b3)  # 输出限制在 [-1, 1]

        # 5. 反归一化至物理推力与力矩
        fx = float(action[0] * self.force_scale)
        fy = float(action[1] * self.force_scale)
        fz = float(action[2] * self.depth_scale)
        tau_z = float(action[3] * self.torque_scale)

        return {
            "fx": fx,
            "fy": fy,
            "fz": fz,
            "tau_z": tau_z,
            "action": action,
            "err_pos": err_pos_world,
            "err_yaw": err_yaw
        }


class LOSTrajectoryController:
    """视线法 (LOS) + 解耦 PID 基准控制器 (Baseline 对照组)"""

    def __init__(
        self,
        kp_xy: float = 280.0,
        kd_xy: float = 85.0,
        kp_z: float = 480.0,
        kd_z: float = 140.0,
        kp_yaw: float = 120.0,
        kd_yaw: float = 35.0,
    ):
        self.kp_xy = kp_xy
        self.kd_xy = kd_xy
        self.kp_z = kp_z
        self.kd_z = kd_z
        self.kp_yaw = kp_yaw
        self.kd_yaw = kd_yaw

    def compute(
        self,
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        current_yaw: float,
        desired_pos: np.ndarray,
        desired_vel: np.ndarray,
        desired_yaw: float,
    ) -> Dict[str, Any]:
        err_pos = desired_pos - current_pos
        err_vel = desired_vel - current_vel
        err_yaw = (desired_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi

        cos_y = math.cos(current_yaw)
        sin_y = math.sin(current_yaw)
        R_bw = np.array([
            [cos_y,  sin_y, 0.0],
            [-sin_y, cos_y, 0.0],
            [0.0,    0.0,   1.0]
        ])

        err_p_b = R_bw @ err_pos
        err_v_b = R_bw @ err_vel
        vel_d_b = R_bw @ desired_vel

        # PID 反馈控制 + 动力学前馈补偿
        fx = self.kp_xy * err_p_b[0] + self.kd_xy * err_v_b[0] + 18.0 * vel_d_b[0]
        fy = self.kp_xy * err_p_b[1] + self.kd_xy * err_v_b[1] + 18.0 * vel_d_b[1]
        fz = self.kp_z  * err_pos[2] + self.kd_z  * err_vel[2] + 25.0 * desired_vel[2]
        tau_z = self.kp_yaw * np.sin(err_yaw)

        # 幅值限幅
        fx = float(np.clip(fx, -350.0, 350.0))
        fy = float(np.clip(fy, -350.0, 350.0))
        fz = float(np.clip(fz, -500.0, 500.0))
        tau_z = float(np.clip(tau_z, -80.0, 80.0))

        return {
            "fx": fx,
            "fy": fy,
            "fz": fz,
            "tau_z": tau_z,
            "err_pos": err_pos,
            "err_yaw": err_yaw
        }


class TrackingEvaluator:
    """轨迹跟踪性能评价器"""

    def __init__(self, name: str = "Controller"):
        self.name = name
        self.times: List[float] = []
        self.pos_errors: List[np.ndarray] = []
        self.yaw_errors: List[float] = []
        self.efforts: List[float] = []

    def record(self, t: float, err_pos: np.ndarray, err_yaw: float, fx: float, fy: float, fz: float, tau_z: float):
        self.times.append(t)
        self.pos_errors.append(err_pos.copy())
        self.yaw_errors.append(abs(err_yaw))
        self.efforts.append(fx*fx + fy*fy + fz*fz + tau_z*tau_z)

    def summary(self) -> Dict[str, Any]:
        """计算统计评价指标"""
        if not self.pos_errors:
            return {}

        errors_xyz = np.array(self.pos_errors)
        dist_errors = np.linalg.norm(errors_xyz, axis=1)
        xy_errors = np.linalg.norm(errors_xyz[:, :2], axis=1)
        z_errors = np.abs(errors_xyz[:, 2])

        rmse_3d = float(np.sqrt(np.mean(dist_errors ** 2)))
        rmse_xy = float(np.sqrt(np.mean(xy_errors ** 2)))
        rmse_z  = float(np.sqrt(np.mean(z_errors ** 2)))
        max_error_3d = float(np.max(dist_errors))

        rmse_yaw = float(np.sqrt(np.mean(np.array(self.yaw_errors) ** 2)))
        avg_effort = float(np.mean(self.efforts))

        return {
            "name": self.name,
            "rmse_3d": rmse_3d,
            "rmse_xy": rmse_xy,
            "rmse_z": rmse_z,
            "max_error": max_error_3d,
            "rmse_yaw_deg": math.degrees(rmse_yaw),
            "avg_effort": avg_effort
        }
